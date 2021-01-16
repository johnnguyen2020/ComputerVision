function plottracks(W)
    for i=1:200
        A(i) = logical(mod(i,2));
        B(i) = not(A(i));
    end
    figure; hold on;
    for i=1:448
        plot(W(A,i),W(B,i));
    end
end
