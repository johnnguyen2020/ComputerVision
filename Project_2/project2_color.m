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
R_matrix = hcd(dest_i, source_i);
ncc_pts = ncc_corners(dest_i, source_i, R_matrix);

%% RANSAC and Homography
inlier_pts = ransac_algo(ncc_pts);

% Optimize the line containing inliers
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
mosaic(S2, S2_color,scale_temp,scale_color);
