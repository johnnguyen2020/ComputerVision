function R_matrix = hcd(Im1, Im2)
I = {Im1, Im2};
deriv_filt =  .5*[-1, 0, 1];
w_size  =3;
window = ones(w_size);
frame_num = 2;

for i = 1:frame_num 
    Dx = conv2(I{i}, deriv_filt', 'same');
    Dy = conv2(I{i}, deriv_filt, 'same');
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

%% Threshold
for i = 1:frame_num
    threshold = max(max(R_matrix(:, :, i)));
    threshold = threshold/2500;
    R_matrix(:, :, i) = R_matrix(:, :, i) .* (R_matrix(:, :, i) > threshold);
end
%% Dialate R matrix and threshold transform the highest value (NMS)
for i = 1:frame_num
   R_dialate = imdilate(R_matrix(:, :, i), ones(7));
   R_matrix(:, :, i) = R_matrix(:, :, i) .* (R_matrix(:, :, i) == R_dialate(:, :));
end