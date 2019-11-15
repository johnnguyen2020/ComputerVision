clear all; close all;

%load frame paths and put in list 
zips1 = ["DanaOffice", "DanaHallWay1", "DanaHallWay2"];
zips2 = ["Cone", "Wall"];

path_folder="/Users/johnnguyen/Documents/CV/john/Project2/"+zips2(1)+"/";
path = '*.jpg';
im_files = dir(path_folder+path);
nfile = length(im_files);

%Select frame indexes from zip file names
dest = 1; source = 2;

%% Store images in I array 
for i = 1:nfile
    RGB = imread(path_folder+im_files(i).name);
    color_images(:,:,:,i) = RGB; 
    I(:, :, i) = rgb2gray(RGB); 
end

dest_i = I(:, :, dest); source_i = I(:, :, source);


%% Disparity Toolbox

disparityRange = [0 128];
disparityMap = disparitySGM(dest_i,source_i,'DisparityRange',disparityRange,'UniquenessThreshold',20);
figure(1);
imshow(disparityMap,disparityRange);
title('Disparity Map');
colormap gray;
colorbar;

%% Hcd and ncc algorithm
% R_matrix = hcd(dest_i, source_i);

Im1 = dest_i;
Im2 = source_i;

I_temp = {Im1, Im2};
deriv_filt =  .5*[-1, 0, 1];
w_size  =3;
window = ones(w_size);
frame_num = 2;

for i = 1:frame_num 
    Dx = conv2(I_temp{i}, deriv_filt', 'same');
    Dy = conv2(I_temp{i}, deriv_filt, 'same');
    Dx2 = Dx.^2; Dy2 = Dy.^2;
    DxDy = Dx .* Dy;
    
    Dx2 = conv2(Dx2, window,'same');
    Dy2 = conv2(Dy2, window,'same');
    DxDy = conv2(DxDy, window,'same');
    
    for x = 1:size(Dx, 1)
        for y = 1:size(Dy, 2)
            c_matrix = [Dx2(x, y), DxDy(x, y);  DxDy(x, y), Dy2(x, y)];
            lambda = eig(c_matrix);
            R_matrix(x,y,i) = (lambda(1)*lambda(2)) - (.04*(lambda(1)+lambda(2))^2);
        end
    end
end

% Threshold using logical greater than 
for i = 1:frame_num
    threshold = max(max(R_matrix(:, :, i)));
    threshold = threshold/2500;
    R_matrix(:, :, i) = R_matrix(:, :, i) .* (R_matrix(:, :, i) > threshold);
end

% Dialate R matrix and threshold transform the highest value (NMS)
for i = 1:frame_num
   R_dialate = imdilate(R_matrix(:, :, i), ones(7));
   R_matrix(:, :, i) = R_matrix(:, :, i) .* (R_matrix(:, :, i) == R_dialate(:, :));
end

%% NCC

%ncc_pts = ncc_corners(dest_i, source_i, R_matrix);
%function ncc_pts = ncc_corners(Im1, Im2,R_matrix)
threshold = .999;
wind_sz = 5;
corners = {};
midpoint = floor(wind_sz/2);
frame_count = 2;
I_temp = {Im1, Im2};
%find the pixels coordinates where corners occur
for i = 1:frame_count
    c = [];
    [c(:,2),c(:,1)] = find(R_matrix(:, :, i) > 0);
    j = c;
    c = c(c(:, 1) < size(I_temp{i},2) - midpoint,:);
    c = c((c(:, 2) < size(I_temp{i},1) - midpoint),:);
    c = c(c(:,1) > midpoint,:);
    c = c(c(:,2) > midpoint,:);
    corners{i} = c;
end


% compute ncc to match pairs of corners
ncc = []; ncc_pts = [];
indexes = -midpoint:midpoint;

for corner_1 = 1:size(corners{1}, 1)
    for corner_2 = 1:size(corners{2}, 1)
        f = double(I_temp{1}(indexes + corners{1}(corner_1,2), indexes + corners{1}(corner_1,1)));
        g = double(I_temp{2}(indexes + corners{2}(corner_2,2), indexes + corners{2}(corner_2,1)));
        f_norm = f/sqrt(sum(sum(f.^2)));
        g_norm = g/sqrt(sum(sum(g.^2)));
        ncc(corner_1,corner_2) = sum(sum((f_norm .* g_norm)));
    end
    [maximum, index_max] = max(ncc(corner_1, :));
    if (maximum > threshold)
        ncc_pts(:, end+1) = [corners{1}(corner_1, 1), corners{1}(corner_1, 2), corners{2}(index_max, 1), corners{2}(index_max, 2), maximum];
    end
end

%% Plot NCC corners uncleaned
figure(3)
imshow([dest_i source_i])
hold on
for i = 1:length(ncc_pts)
    hold on
    plot([ncc_pts(1,i) ncc_pts(3,i)+size(dest_i,2)],[ncc_pts(2,i) ncc_pts(4,i)])
end

%% RANSAC and Homography
%inlier_pts = ransac_algo(ncc_pts);
%function inlier_indicies = ransac_algo(ncc_pts)
% Homography
trials = 3000;
inlier_threshold = 0.00001;

input_corners = []; 
output_corners = [];
max_count = 0;
h_gold = [];
inlier_indicies = [];
constraints = [];
for i = 1:trials
    %pick four random distinct points
    four_crns = randperm(size(ncc_pts, 2), 8);
    input_corners(:, 1) = ncc_pts(1, four_crns);
    input_corners(:, 2) = ncc_pts(2, four_crns);
    input_corners(:,3) = ones(1,8);
    
    output_corners(:, 1) = ncc_pts(3, four_crns);
    output_corners(:, 2) = ncc_pts(4, four_crns);
    output_corners(:,3) = ones(1,8);

    %compute homography 
    %h = homography(input_corners, output_corners);
    %h = reshape(h, [3, 3])';
    a = input_corners;
    b = output_corners;
    
    %Fundamental Matrix Algorithm 
    p = [];
    for i = 1:size(a, 1)
        p = vertcat(p,  [a(i, 1)*b(i,1) a(i, 1)*b(i,2) a(i,1) a(i,2)*b(i,1) b(i,2)*a(i,2) a(i,2) b(i,1) b(i,2) 1]);
    end
    
    [U,S,V] = svd(p);
    [~, min_index] = min(diag(S));
    D = V(: , min_index);
    S(min_index,min_index) = 0;
    [U_,S_,V_] = svd(U*S*V');
    [~, min_index] = min(diag(S));
    D = V(: , min_index);
    h = reshape(D,[3,3]);
    
    for i = 1:size(ncc_pts,2)
        constraints(i) = [ncc_pts([3,4],i);1]' * h * [ncc_pts([1,2],i);1];
    end
    
    inlier_count = sum(abs(constraints(:)) < inlier_threshold);
   
    %check and update for most inliers
    if max_count < inlier_count
        max_count = inlier_count;
        inlier_indicies = abs(constraints) < inlier_threshold;
        h_gold = h;
    end
end

inlier_pts = inlier_indicies;


%% Optimize the line containing inliers
source_corners(:, 1) = ncc_pts(1, inlier_pts);
source_corners(:, 2) = ncc_pts(2, inlier_pts);
dest_corners(:, 1) = ncc_pts(3, inlier_pts);
dest_corners(:, 2) = ncc_pts(4, inlier_pts);

%h = homography(source_corners, dest_corners);

%     a = input_corners;
%     b = output_corners;
    
    %Fundamental Matrix Algorithm 
    lp = [];
    for i = 1:size(input_corners, 1)
        lp = vertcat(lp,  [input_corners(i, 1)*output_corners(i,1) input_corners(i, 1)*output_corners(i,2) input_corners(i,1) input_corners(i,2)*output_corners(i,1) output_corners(i,2)*input_corners(i,2) input_corners(i,2) output_corners(i,1) output_corners(i,2) 1]);
    end
    
    [U,S,V] = svd(lp);
    [~, min_index] = min(diag(S));
    D = V(: , min_index);
    S(min_index,min_index) = 0;

    [U_,S_,V_] = svd(U*S*V');
    [~, min_index] = min(diag(S));
    D = V(: , min_index);
    h = reshape(D,[3,3]);

%% Plot optimal homography results 
figure
imshow([dest_i source_i])
hold on

% Plot corners on image
for i = 1:length([source_corners(:, 1)'])
    source1 = source_corners(:, 1)';
    source2 = dest_corners(:, 1)' + size(dest_i, 2);
    dest1 = source_corners(:, 2)';
    dest2 = dest_corners(:, 2)';
    plot([source1(i), source2(i)], [dest1(i), dest2(i)]);
end

%% Disparity map Vectors
disp_thresh = [60 0];
window_sz = 5;
[X,Y] = meshgrid(1+floor(window_sz/2):size(dest_i,2)-(floor(window_sz/2)+1),1+floor(window_sz/2):size(dest_i,1)-(floor(window_sz/2)+1));
dispX = zeros(size(X));
dispY = zeros(size(Y));
for i = 1:length(unique(Y(:)))
    for j = 1:length(unique(X(:)))
        pt = [X(i,j),Y(i,j),1];
        epiline = pt * h;
        [x_l,y_l] = ncc_find(dest_i,source_i,window_sz,pt,epiline,unique(X(:)));
        pt_2 = [x_l,y_l,1];
        disparity = pt - pt_2;
        if disparity(1) < disp_thresh(1)
            dispX(i,j) = disparity(1);
        else
            dispX(i,j) = disp_thresh(1);
        end
        if disparity(2) > disp_thresh(2)
            dispY(i,j) = disparity(2);
        else
            dispY(i,j) = disp_thresh(2);
        end
    end
end
%% Plot Disparity Vectors
figure
subplot(1,2,1);
imshow(uint8(dispX))
title('Horizontal Disparity')
subplot(1,2,2);
imshow(uint8(dispY))
title('Vertical Disparity')

%% Disparity Color Map
figure
mag = sqrt(dispX.^2 + dispY.^2); 
mu = mean(mag(:));
std_ = std(mag(:));
mag = (mag-mu)/std_;
color_im = zeros([size(dispX), 3]);

for i = 1:size(dispX,1) 
    for j = 1:size(dispX,2) 
        orientation = (atan2d(dispY(i, j),(dispX(i, j))) + 180)/360;      
        if isnan(orientation)
           if dispY(i,j) >= 0
               orientation = 90/360;
           else 
               orientation = 270/360;
           end
        end
        color_rgb = hsv2rgb(orientation, mag(i, j), 1);
        color_im(i, j, :) = color_rgb;
    end
end

imshow(color_im);

%% Nomalized Gray 
normalizedImage = uint8(255*mat2gray(color_im));
figure;
imshow(normalizedImage);


