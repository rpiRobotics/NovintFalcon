function res=R2Q(R)
%R2q calculate the quaternion from the rotation matrix
%R is the rotation matrix
%res is a 4x1 vector that includes the quaternion. res1 is cos(theta/2)
%This version is modified on Mar 23rd. The equation comes from spacecraft
%notes 4. 
%The rotation matrix is the transpose form of the roation matrix introduced in robotics class.
%it's the rotation of initial frame in the final frame
res=zeros(4,1);
res(1)=sqrt(1+R(1,1)+R(2,2)+R(3,3))/2;

res(2)=(R(3,2)-R(2,3))/(4*res(1));
res(3)=(R(1,3)-R(3,1))/(4*res(1));
res(4)=(R(2,1)-R(1,2))/(4*res(1));

res=res/norm(res);
end
