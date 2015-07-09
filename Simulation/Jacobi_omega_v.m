function J = Jacobi_omega_v(theta,vec,vectransl)
n=length(theta);
theta=reshape(theta,1,n);
R{1}=eye(3);
q{1}=[1;0;0;0];
pBase2Tip{1}=[0;0;0];
for i=1:n
    q{i+1}=QuaternionMultiply([cos(theta(i)/2);R{i}*vec(:,i)*sin(theta(i)/2)],q{i}); %q{i+1} denotes the quaternion q_{B->{i}} in Base frame
    pBase2Tip{i+1}=pBase2Tip{i}+R{i}*vectransl(:,i); %pBase2Tip{i+1} is vector pointing from origin to {i}
    R{i+1}=R{i}*q2R([cos(theta(i)/2);vec(:,i)*sin(theta(i)/2)]); %R{i+1} denotes the rotation matrix R_{B->{i}}.
end
pBase2Tip{n+2}=pBase2Tip{n+1}+R{n+1}*vectransl(:,n+1);
J=[];
for i=1:n
    h{i}=R{i+1}*vec(:,i);
    p_end{i}=pBase2Tip{n+2}-pBase2Tip{i+1};
    J(:,i)=[h{i};cross(h{i},p_end{i})];
end
end