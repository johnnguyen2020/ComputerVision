clear all; close all;

%load frame paths and put in list 
zips = ["DanaOffice", "DanaHallWay1", "DanaHallWay2"];
path_folder="/Users/johnnguyen/Documents/CV/john/Project2/"+zips(3)+"/";
path = '*.jpg';
im_files = dir(path_folder+path);
nfile = length(im_files);

%Select frame indexes from zip file names
dest = 2; source = 1;

%% Store images in I array 
for i = 1:nfile
    RGB = imread(path_folder+im_files(i).name);
    color_images(:,:,:,i) = RGB; 
    I(:, :, i) = rgb2gray(RGB); 
    %figure(i);
    %imshow(I(:, :, i));
    %hold on
end

dest_i = I(:, :, dest); source_i = I(:, :, source);

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
threshold = .98;
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

%% RANSAC and Homography
%inlier_pts = ransac_algo(ncc_pts);
%function inlier_indicies = ransac_algo(ncc_pts)
% Homography
trials = 146;
inlier_threshold = 5;

input_corners = []; output_corners = [];
max_count = 0;
h_gold = [];
inlier_indicies = [];
for i = 1:trials
    %pick four random distinct points
    four_crns = randperm(size(ncc_pts, 2), 4);
    input_corners(:, 1) = ncc_pts(1, four_crns);
    input_corners(:, 2) = ncc_pts(2, four_crns);
    output_corners(:, 1) = ncc_pts(3, four_crns);
    output_corners(:, 2) = ncc_pts(4, four_crns);
    
    %compute homography 
    h = homography(input_corners, output_corners);
    h = reshape(h, [3, 3])';
    
    %Threshold margin for ransac
    input_corners_shift = h * [input_corners, ones(4, 1)]';
    input_corners_plane_shift(1, :) = input_corners_shift(1, :) ./ input_corners_shift(3, :);
    input_corners_plane_shift(2, :) = input_corners_shift(2, :) ./ input_corners_shift(3, :);
    computed_corners = h * [ncc_pts(1:2, :); ones(size(ncc_pts, 2), 1)'];
    computed_corner_plane(1, :) = computed_corners(1, :) ./ computed_corners(3, :);
    computed_corner_plane(2, :) = computed_corners(2, :) ./ computed_corners(3, :);
    %count 
    l2_shift = sqrt(sum((computed_corner_plane' - ncc_pts(3:4, :)').^2, 2));
    inlier_count = sum(l2_shift < inlier_threshold);
    
    %check and update for most inliers
    if max_count < inlier_count
        max_count = inlier_count;
        inlier_indicies = l2_shift < inlier_threshold;
        h_gold = h;
    end
end

inlier_pts = inlier_indicies;


%% Optimize the line containing inliers
source_corners(:, 1) = ncc_pts(1, inlier_pts);
source_corners(:, 2) = ncc_pts(2, inlier_pts);
dest_corners(:, 1) = ncc_pts(3, inlier_pts);
dest_corners(:, 2) = ncc_pts(4, inlier_pts);
h = homography(source_corners, dest_corners);

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

%% Transform Frame
i_dim = size(I)
h = reshape(h, [3, 3])';
h_inv = inv(h);

edge_points = [1 1 1; 1, i_dim(1), 1; i_dim(2) i_dim(1) 1; i_dim(2) 1, 1]'
edge_transform = h * edge_points;
edge_transform(1, :) = edge_transform(1, :) ./ edge_transform(3, :);
edge_transform(2, :) = edge_transform(2, :) ./ edge_transform(3, :);
[max_x] = max(edge_transform(1, :));
[max_y] = max(edge_transform(2, :));
[min_x] = min(edge_transform(1, :));
[min_y] = min(edge_transform(2, :));

[xx, yy] = meshgrid((1:max_x), (min_y:max_y));

XX = (h_inv(1,1)*xx+h_inv(1,2)*yy+h_inv(1,3))./(h_inv(3,1)*xx+h_inv(3,2)*yy+h_inv(3,3));
YY = (h_inv(2,1)*xx+h_inv(2,2)*yy+h_inv(2,3))./(h_inv(3,1)*xx+h_inv(3,2)*yy+h_inv(3,3));

scale = interp2(double(dest_i), XX, YY);
scale_temp = scale;
%scale(isnan(scale)) = 0;

% Transformation of color frames
for i = 1:3
    scale_color(:,:,i) = interp2(double(color_images(:,:,i,dest)), XX, YY);
end

[XX YY] = meshgrid(1:max_x,min_y:max_y);
S2 = interp2(double(source_i), XX, YY);

for i = 1:3
    S2_color(:,:,i) = interp2(double(color_images(:,:,i,source)), XX, YY);
end

%%
%mosaic(S2, S2_color,scale_temp,scale_color);
%function mosaic(S2, S2_color,scale_temp,scale_color)

[rowS2,colS2] = find(~isnan(S2_color));
[rowscale,colscale] = find(~isnan(scale_color));
end_col = max(colS2);
begin_col = min(colscale);
merged = S2_color+scale_color;
merged(:,begin_col:end_col) = merged(:,begin_col:end_col)/1.8;
merged(isnan(scale_color)) = S2_color(isnan(scale_color));
merged(isnan(S2_color)) = scale_color(isnan(S2_color));
figure
imshow(merged/255)

