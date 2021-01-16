function Q = solveQ(U)
  H = []; y = [];
  for i = 1:100
      I = U(2*i-1,:);
      J = U(2*i,:);
      II = reshape(I'*I,1,9);
      JJ = reshape(J'*J,1,9);
      IJ = reshape(I'*J,1,9);
      JI = reshape(J'*I,1,9);
      H = [H;II;JJ;IJ;JI];
      y = [y;1;1;0;0];
  end
  x = inv(H'*H)*H'*y;
  R = reshape(x, 3, 3)';
  Q = chol(R)';
end

