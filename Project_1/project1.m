%% Load in folder path, get list of filenames
clear all
close all
path="/Users/johnnguyen/Downloads/RedChair";
im_files = dir(path);
im_files = {im_files.name};

%% Load in files to array 
image_dim = size(imread(path+"/"+im_files{24}));
images = zeros(image_dim(1),image_dim(2),length(im_files)-2);
nfile = length(im_files);
%for some reason the first two values are . and .. in filename list
for i = 3:nfile
images(:,:,i-2) = rgb2gray(imread(path+"/"+im_files{i}));
end

%% keep my color images
%images_color = zeros(image_dim(1),image_dim(2),length(im_files)-2);
%for i = 3:length(im_files)
%images_color(:,:,(i-2):i-2+2) =imread(path+"/"+im_files{i});
%end

%% get differences between photos
images_dim = size(images); %returns 240 320 353
temporal_differences = zeros(image_dim(1),image_dim(2),images_dim(3));
temporal_differences_abs = zeros(image_dim(1),image_dim(2),images_dim(3));

for i = 3:images_dim(3)-1
    temporal_differences(:,:,i) = (images(:,:,i+1) - images(:,:,i-1));
    temporal_differences_abs(:,:,i) = abs(images(:,:,i+1) - images(:,:,i-1));
end

%edge cases
temporal_differences(:,:,1) = images(:,:,2) - images(:,:,1);
temporal_differences(:,:,images_dim(3)) = images(:,:,images_dim(3)) - images(:,:,images_dim(3)-1);

temporal_differences_abs(:,:,1) = images(:,:,2) - images(:,:,1);
temporal_differences_abs(:,:,images_dim(3)) = images(:,:,images_dim(3)) - images(:,:,images_dim(3)-1);

%% display video of temporal difference along frames
figure
for i = 1:length(temporal_differences)
    if i == 65 | i == 164 | i ==218 | i == 348
        subplot(1,2,1)
        imshow(temporal_differences(:,:,i))
        subplot(1,2,2)
        imshow(temporal_differences_abs(:,:,i))
        w = waitforbuttonpress
    end
end
close all

%% get threshold and create mask
threshold = mean(temporal_differences_abs(:)) + 2*std(temporal_differences_abs(:));
%logical mask
mask = temporal_differences_abs > threshold;
%% apply mask to images
masked_images = images/255 .* mask;

%%
figure
for i = 1:length(mask)
    if i == 65 | i == 164 | i ==218 | i == 348
        subplot(1,3,1)
        imshow(images(:,:,i)/255)
        title('images')

        subplot(1,3,2)
        imshow(mask(:,:,i))
        title('mask')

        subplot(1,3,3)
        imshow(masked_images(:,:,i))
        title('overlay')
        pause(.02)
        w = waitforbuttonpress
    end 
end

close all