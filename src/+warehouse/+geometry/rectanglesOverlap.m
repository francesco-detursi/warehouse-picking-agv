function overlap = rectanglesOverlap(a,b)
overlap = a(1)<b(1)+b(3) && a(1)+a(3)>b(1) && ...
    a(2)<b(2)+b(4) && a(2)+a(4)>b(2);
end
