function y = QuaternionBase2Tip(theta,vec)
n=length(theta);
R{1}=eye(3);
q{1}=[1;0;0;0];
for i=1:n
    q{i+1}=QuaternionMultiply([cos(theta(i)/2);R{i}*vec(:,i)*sin(theta(i)/2)],q{i}); %q{i+1} denotes the quaternion q_{B->{i}} in Base frame
    R{i+1}=R{i}*q2R([cos(theta(i)/2);vec(:,i)*sin(theta(i)/2)]); %R{i+1} denotes the rotation matrix R_{B->{i}}.
end
y=q{n+1};
end