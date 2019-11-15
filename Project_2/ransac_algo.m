function inlier_indicies = ransac_algo(ncc_pts)
%% Homography
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