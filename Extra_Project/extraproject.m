clear all; close all; clc;

%% load images
init = imread("hotel/hotel.seq0.png");
for i = 1:100
    hotel{i} = imread("hotel/hotel.seq"+i+".png");
end

%% init tracker
corners = hcd(init);
tracker = vision.PointTracker('MaxBidirectionalError',1);
initialize(tracker,corners,init);

jf = 21
figure, imshow(hotel{jf}), hold on, scatter(pts{jf}(:,1),pts{jf}(:,2)), hold off;

%% get measurement matrix
cornercnt = size(corners,1);
valid = true(cornercnt,1);
for i = 1:100
    [points,validity] = tracker(hotel{i});
    valid = valid & validity;
    pts{i} = points;
end
for i = 1:100
    pts{i} = pts{i}(valid, :);
    pts_avg{i} = pts{i} - mean(pts{i});
end
W = [];
for i = 1:100
    W = [W;pts_avg{i}'];
end

plottracks(W)
%% compute Q
[U,D,V] = svd(W);
M = U*sqrt(D); M = M(:,1:3);
S = sqrt(D)*V; S = S(1:3,:);
Q = solveQ(M);
P = inv(Q)*S;
scatter3(P(1,:),P(2,:),P(3,:));
