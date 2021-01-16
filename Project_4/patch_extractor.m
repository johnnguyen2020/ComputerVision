function [image_patches,xywh,xy] = patch_extractor(RGB,Corners,r_size)
    image_patches = [];
    xywh = [];
    xy = [];
    for c = Corners'
       y = c(1);
       x = c(2);
       try
           patch = RGB((x-r_size):(x+r_size),(y-r_size):(y+r_size));
           bx = y-r_size/2;
           by = x-r_size/2;
           box = [bx,by,r_size,r_size];
       catch
           continue
       end
       image_patches = cat(3,image_patches,patch);
       xywh = cat(1,xywh,box);
       xy = cat(1,xy,[y,x]);
    end
end