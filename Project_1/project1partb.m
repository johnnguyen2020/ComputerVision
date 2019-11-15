%% load in image
clear all; close all;

path_folder="/Users/johnnguyen/Documents/MATLAB/RedChair/";
path = '*.jpg';
image_files = dir(path_folder+path);
nfile = length(image_files);


%% put images in array

%define filters
smoothing_filter_1 = fspecial('average',[3,3]);
smoothing_filter_2 = fspecial('average',[5,5]);

tsigma = 3;
smoothing_filter_1 = fspecial('gaussian',[ceil((5*tsigma)),ceil((5*tsigma))],tsigma);

%select smoothing filter
smoothing_bool = false;
smoothing_filter = smoothing_filter_2;

for i = 1:nfile
    %image_files(i).name;
    %Read in images to grayscalse
    RGB = imread(path_folder+image_files(i).name);
    I(:,:,i) = rgb2gray(RGB);

    if smoothing_bool
        % smooth images before processing, replicate border 
        I(:,:,i) = imfilter(I(:,:,i),smoothing_filter,'replicate');
    end
    
end

image_dimension = size(I);
%% apply temporal derivative 
temporal_derivative = zeros(image_dimension(1),image_dimension(2),nfile);

tsigma = 1.4;
derivative_simple = .5*[-1,0,1];
%algorithm to compute derivative of gaussian 
derivative_gaussian = conv(fspecial('gaussian',[1,(5*tsigma)],tsigma),derivative_simple,'same');

%select derivative type
derivative = derivative_gaussian;

%derivative = [-1,0,1]
for i = 1:image_dimension(1)
    for j = 1:image_dimension(2)
        %temporal_derivative(i,j,1:end) = imfilter(reshape(I(i,j,1:end),1,nfile,1),derivative,'replicate');
        %reshape list to pixel projection across all frames
        temporal_derivative(i,j,1:end) = conv(reshape(I(i,j,1:end),1,nfile,1),derivative,'same');

    end
end

%% Select threshold values
%take mean in temporal dimension
mean_pixel = mean(abs(temporal_derivative),3);
mean_threshold = mean(mean_pixel(:));

std_pixel = std(abs(double(temporal_derivative)),0,3);
std_threshold = mean(std_pixel(:));

mask = (abs(temporal_derivative) > mean_threshold + 2 * std_threshold);
%% Salt and pepper filter to mask
% for i = 1:nfile
% mask(:,:,i) = medfilt2(mask(:,:,i),[3,3]); 
% end
%% convert images to RGB
path = '*.jpg';
image_files = dir(path_folder+path);

nfile = length(image_files);

for i = 1:nfile
    image_files(i).name;
    RGB(:,:,:,i) = imread(path_folder+image_files(i).name);
end

%% overlay mask on top of RGB
mask_3D = permute(cat(4,mask,mask,mask),[1 2 4 3]);
for i = 1:nfile 
    image_overlay(:,:,:,i) = imoverlay(RGB(:,:,:,i),mask(:,:,i),'yellow');
    image_overlay2(:,:,:,i) = uint8(RGB(:,:,:,i)) .* uint8(mask_3D(:,:,:,i));
    image_overlay3(:,:,:,i) = imoverlay(temporal_derivative(:,:,i),mask(:,:,i),'green');
end

%%
figure
for i = 1:length(mask)
    
    if i == 65 | i == 164 | i ==218 | i == 349
        subplot(1,3,1)
        imshow(RGB(:,:,:,i))
        title('RGB')
        
        subplot(1,3,2)
        imshow(uint8(mask_3D(:,:,:,i))*255)
        title('Mask') 

        subplot(1,3,3)
        imshow(image_overlay(:,:,:,i))
        title('Overlay')

%         subplot(1,4,3)
%         imshow(image_overlay2(:,:,:,i))
%         title('Smoothed')


        w = waitforbuttonpress;
    end 
end

close all