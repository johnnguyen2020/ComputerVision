function ncc_pts = ncc_corners(Im1, Im2,R_matrix)
threshold = .98;
wind_sz = 5;
corners = {};
midpoint = floor(wind_sz/2);
frame_count = 2;
I = {Im1, Im2};
%find the pixels coordinates where corners occur
for i = 1:frame_count
    c = [];
    [c(:,2),c(:,1)] = find(R_matrix(:, :, i) > 0);
    c = c(c(:, 1) < size(I{i},2) - midpoint,:);
    c = c((c(:, 2) < size(I{i},1) - midpoint),:);
    c = c(c(:,1) > midpoint,:);
    c = c(c(:,2) > midpoint,:);
    corners{i} = c;
end

% compute ncc to match pairs of corners
ncc = []; ncc_pts = [];
indexes = -midpoint:midpoint;

for corner_1 = 1:size(corners{1}, 1)
    for corner_2 = 1:size(corners{2}, 1)
        f = double(I{1}(indexes + corners{1}(corner_1,2), indexes + corners{1}(corner_1,1)));
        g = double(I{2}(indexes + corners{2}(corner_2,2), indexes + corners{2}(corner_2,1)));
        f_norm = f/sqrt(sum(sum(f.^2)));
        g_norm = g/sqrt(sum(sum(g.^2)));
        ncc(corner_1,corner_2) = sum(sum((f_norm .* g_norm)));
    end
    [maximum, index_max] = max(ncc(corner_1, :));
    if (maximum > threshold)
        ncc_pts(:, end+1) = [corners{1}(corner_1, 1), corners{1}(corner_1, 2), corners{2}(index_max, 1), corners{2}(index_max, 2), maximum];
    end
end
