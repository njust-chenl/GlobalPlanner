function n_index = node_index(OPEN,xval,yval)
    %This function returns the index of the location of a node in the list
    %OPEN      在OPEN list容器里面找到目标点的坐标（x，y）然后返回这个点在open list里面的行数。
    %
    %   Copyright 2009-2010 The MathWorks, Inc.
%     i=1;
%     while(OPEN(i,2) ~= xval || OPEN(i,3) ~= yval )
%         i=i+1;
%   
%     end
%     n_index=i;
 xFind = find((OPEN(:,2)==xval));
    yFind = find((OPEN(:,3)==yval));
    n_index = intersect(xFind,yFind);
end