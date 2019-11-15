function h = homography(in, out)
a = [];
for i = 1:size(in, 1)
    a = vertcat(a,  [ in(i, 1) in(i, 2) 1 0 0 0 -in(i, 1)*out(i, 1) -in(i, 2) * out(i, 1) -out(i, 1);
    0 0 0 in(i, 1) in(i, 2) 1 -in(i, 1)*out(i, 2) -in(i, 2)*out(i, 2) -out(i, 2)]);
end

[U,S,~] = svd(a'*a);
[~, mini] = min(diag(S));
h = U(: , mini);
%size(h)
%h = reshape(h, [3, 3])';

end