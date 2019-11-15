function mosaic(S2, S2_color,scale_temp,scale_color)
[rowS2,colS2] = find(~isnan(S2_color));
[rowscale,colscale] = find(~isnan(scale_color));
end_col = max(colS2);
begin_col = min(colscale);
merged = S2_color+scale_color;
merged(:,begin_col:end_col) = merged(:,begin_col:end_col)/1.8;
merged(isnan(scale_color)) = S2_color(isnan(scale_color));
merged(isnan(S2_color)) = scale_color(isnan(S2_color));
figure
imshow(merged/255)
end
