function dmin=Dist_PointToLineseg(P1,P2,P3)
%calculate minimum distance from P3 to line segment P1P2
x1=P1(1);y1=P1(2);x2=P2(1);y2=P2(2);x3=P3(1);y3=P3(2);
if (x3-x1)*(x2-x1)+(y3-y1)*(y2-y1)<0 
    dmin=sqrt((x3-x1)^2+(y3-y1)^2);
elseif (x3-x2)*(x1-x2)+(y3-y2)*(y1-y2)<0
    dmin=sqrt((x3-x2)^2+(y3-y2)^2);
else
    dmin=abs((y2 - y1) * x3 +(x1 - x2) * y3 + ((x2 * y1) -(x1 * y2))) / sqrt((y2 - y1)^2 + (x1 - x2)^2);
end

