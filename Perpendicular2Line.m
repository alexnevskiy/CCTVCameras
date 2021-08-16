function [fP,D,outside]=Perpendicular2Line(P,sP,eP)
%Calculate the foot points fP and the minimal distances D
%from a point P to a 2D or 3D line segment.
%INPUT:
  %P : Reference point,  P=[xr,xr] or  P=[xr,yr,zr]
  %sP: Starting points, sP=[Xs,Ys] or sP=[Xs,Ys,Zs]
  %eP: Ending points,   eP=[Xe,Ye] or eP=[Xe,Ye,Ze]
%OUTPUT:
  %fP: Foot points X,Y where the perpendiculars hits the line segments
    %If perpendicular is outside of line segment, take closest data point.
  %D : Distances from P to line segments
  %outside: Indicator for pependicular position
    %outside 0: Perpendicular inside line segment
    %outside 1: Perpendicular outside start of line segment
    %outside 2: Perpendicular outside end of line segment
%EXAMPLE (2D): 
  %P=[0,0];sP=[-4,1;-4,2;-1,2];eP=[-2,1;-2,4;1,2];
  %[fP,D,outside]=Perpendicular2Line(P,sP,eP)
  
%Author: Peter Seibold, December 2020
%Basic idea from Ken Eaton, https://stackoverflow.com/questions/43991659/how-do-i-compute-the-shortest-distance-from-a-point-to-line-segments

[size1sP,size2sP]=size(sP);
P = repmat(P, [size1sP 1]);
vec = eP-sP;
u = sum(vec.*(P-sP), 2)./sum(vec.*vec, 2);
u(isnan(u))=0;%u=0/0=NaN, lines with zero length -> only one point
if size2sP==2
  %2D
  fP = sP+[u u].*vec;%Foot points
else
  %3D
  fP = sP+[u u u].*vec;%Foot points
end
outside=zeros(size1sP,1);
outside(u<0)=1;%Outside of starting point, fP takes starting point
outside(u>1)=2;%Outside of ending point, fP takes ending point
fP(u < 0, :) = sP(u < 0, :);%Outside of starting point, take starting point
fP(u > 1, :) = eP(u >1, :);%Outside of end point, take end point
%Distances P to foot points
if size2sP==2
  %2D
  D=sqrt((P(:,1)-fP(:,1)).^2+(P(:,2)-fP(:,2)).^2);
else
  %3D
  D=sqrt((P(:,1)-fP(:,1)).^2+(P(:,2)-fP(:,2)).^2+(P(:,3)-fP(:,3)).^2);
end
