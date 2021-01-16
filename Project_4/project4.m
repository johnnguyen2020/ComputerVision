clear all; close all;

%load frame paths and put in list 
zips = ["CarTestImages", "CarTrainImages"];
test_folder="/Users/johnnguyen/Documents/CV/Projects/Project4/"+zips(1)+"/";
train_folder="/Users/johnnguyen/Documents/CV/Projects/Project4/"+zips(2)+"/";
load('/Users/johnnguyen/Documents/CV/Projects/Project4/GroundTruth/CarsGroundTruthBoundingBoxes.mat')


path = '*.jpg';
im_test = dir(test_folder+path);
im_train = dir(train_folder+path);
numTest = length(im_test);
numTrain = length(im_train);

window_size=7;

for i = 1:numTrain
    fprintf("Training example %d loaded\n",i)
    RGB = imread(train_folder+im_train(i).name);
    train_data(i).OG_RGB = RGB;
    c = detectHarrisFeatures(RGB);
    train_data(i).RGB = RGB;
    [patches,xywh,xy] = patch_extractor(RGB,round(c.Location),window_size);
    train_data(i).corner_coordinates = xy;
    train_data(i).patches = patches;
    train_data(i).xywh = xywh;
end
center_row = size(RGB,1)/2;
center_col = size(RGB,2)/2;

% if want to concatenate all images along 3rd dimension... then cat(3,train_data.RGB)
%%
for i = 1:9
    RGB = train_data(i).OG_RGB;
    X = train_data(i).corner_coordinates;
    patches = train_data(i).patches;
    patch_xywh = train_data(i).xywh;
    figure(1)
    subplot(3,3,i)
    imshow(RGB)
    hold on 
    plot(X(:,1),X(:,2),'y*')

    % plot corners
    for xywh = patch_xywh'
        rectangle('Position',xywh)
    end
end
%% K-means
patches = double(cat(3,train_data.patches));

% resize patches for k-means
patches = reshape(patches,(window_size*2+1)*(window_size*2+1),size(patches,3));

k_fit = [];
for k = 1:15
    [~,~,sumd] = kmeans(patches',k);
    k_fit = [k_fit mean(sumd)];
end
%% elbow inspection
figure(2)
plot(1:15,k_fit)
title("Elbow Inspection")
xlabel("K-clusters")
% by visual inspection I think k=4 is ideal
%% refit K-means with ideal 
K = 6
[c_idx,c_mean,~,D] = kmeans(patches',K);
%% visualize patches
for patch = c_mean'
    figure
    imshow(reshape(uint8(patch),(window_size*2+1),(window_size*2+1)))
end
%% assign labels to patches
n_start = 1;
for i = 1:numTrain
    n_patches = size(train_data(i).patches,3);
    train_data(i).vocab_idx = c_idx(n_start:(n_start+n_patches-1));
    n_start = n_start + n_patches;
end
%% record distances to center
for i = 1:numTrain
    train_data(i).displacement_vecs = train_data(i).corner_coordinates-[center_col,center_row]
end
%% displacement table
for k = 1:K
    idx = c_idx==k
    displacement_vec = cat(1,train_data.displacement_vecs);
    displacement_table{k} = displacement_vec(idx,:);
end
%% test time
for i = 1:numTest
    fprintf("Testing example %d loaded\n",i)
    RGB = imread(test_folder+im_test(i).name);
    test_data(i).OG_RGB = RGB;
    c = detectHarrisFeatures(RGB);
    test_data(i).RGB = RGB;
    [patches,xywh,xy] = patch_extractor(RGB,round(c.Location),window_size);
    test_data(i).corner_coordinates = xy;
    test_data(i).patches = patches;
    test_data(i).xywh = xywh;
end
center_row = size(RGB,1)/2;
center_col = size(RGB,2)/2;
% if want to concatenate all images along 3rd dimension... then cat(3,train_data.RGB)
%% assign labels to patches
for i = 1:numTest
    fprintf("Test point assignment %d\n",i)
    patch_img = test_data(i).patches;
    vocab_idx = [];
    for j = 1:size(patch_img,3)
        distances = sum(sqrt((c_mean-double(reshape(patch_img(:,:,j),1,(window_size*2+1)*(window_size*2+1)))).^2),2);
        [min_distance,k_pred] = min(distances);
        vocab_idx = [vocab_idx k_pred];
    end
    test_data(i).vocab_idx = vocab_idx;
end
%% voting time
tally = 0
for i = 1:numTest
    img = test_data(i);
    voting_scheme = zeros(size(img.RGB));
    xy = img.corner_coordinates;
    vocab_id = img.vocab_idx;
    for j = 1:size(xy,1)
        r = xy(j,2);
        c = xy(j,1); 
        displace = displacement_table{vocab_id(j)};
        r = r + displace(:,2);
        c = c + displace(:,1);
        erase = ((r>0)&(c>0)) & (c<size(voting_scheme,2)) & (r<size(voting_scheme,1));
        idx = sub2ind(size(voting_scheme),r(erase),c(erase));
        voting_scheme(idx) = voting_scheme(idx)+1;
    end
    close all
    figure(3)
    minimum = min(voting_scheme(:));
    [maximum,idx] = maxk(voting_scheme(:),10);
    maximum = max(maximum);
    
    [y,x] = ind2sub(size(voting_scheme),idx);
    subplot(1,2,1)
    imshow(img.RGB)
    hold on
    plot(x,y,'y*')
    
    rectangle('Position',[max([x-50 y-20]) 100 40],'EdgeColor', 'b')

   
    for p =1:size(x,1)
        if abs(max(x)-x(p)) > 50 | abs(max(y)-y(p)) > 30 
            rectangle('Position',[x(p)-50 y(p)-20 100 40],'EdgeColor', 'b')
            break
        end
    end
    
    for k = 1:size(groundtruth(i).topLeftLocs ,1)
        rectangle('Position',[groundtruth(i).topLeftLocs(k,:),groundtruth(i).boxW, groundtruth(i).boxH],'EdgeColor', 'y')
    end
    
    %overlap results
    A = [max([x-50 y-20]) 100 40];
    B = [groundtruth(i).topLeftLocs(1,:),groundtruth(i).boxW, groundtruth(i).boxH]
    area = rectint(A,B);
    den = ((100*40*2)-(area*2))
    iou = round(area/den,2)
    if iou > .5
        tally = tally + 1;
    end
    subplot(1,2,2)                                     
    imshow(voting_scheme/maximum)  
%     subplot(1,3,3)
%     imshow(img.RGB)
%     hold on
%     
%     for k = 1:size(groundtruth(i).topLeftLocs ,1)
%         rectangle('Position',[groundtruth(i).topLeftLocs(k,:),groundtruth(i).boxW, groundtruth(i).boxH],'EdgeColor', 'y')
%     end
    %pause   
end

result = tally/numTest
%% 
figure
img = test_data(4)
imshow(img.RGB)
hold on
plot(xy(:,1),xy(:,2),'y*')

%% plot ground truth boxes on test data
for i = 1:numTest
    figure
    imgTest = test_data(i)
    imshow(imgTest.RGB)
    hold on
    for k = 1:size(groundtruth(i).topLeftLocs ,1)
        rectangle('Position',[groundtruth(i).topLeftLocs(k,:),groundtruth(i).boxW, groundtruth(i).boxH],'EdgeColor', 'y')
    end 
    pause
end
    




