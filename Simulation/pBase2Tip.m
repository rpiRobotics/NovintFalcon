function y = pBase2Tip(theta,vec,vectransl)
n=length(theta);
R{1}=eye(3);
q{1}=[1;0;0;0];
pBase2Tip=[0;0;0];
for i=1:n
    q{i+1}=QuaternionMultiply([cos(theta(i)/2);R{i}*vec(:,i)*sin(theta(i)/2)],q{i}); %q{i+1} denotes the quaternion q_{B->{i}} in Base frame
    pBase2Tip=pBase2Tip+R{i}*vectransl(:,i);
    R{i+1}=R{i}*q2R([cos(theta(i)/2);vec(:,i)*sin(theta(i)/2)]); %R{i+1} denotes the rotation matrix R_{B->{i}}.
end
pBase2Tip=pBase2Tip+R{n+1}*vectransl(:,n+1);
y=pBase2Tip;
end