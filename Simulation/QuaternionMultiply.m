function y = QuaternionMultiply(p,q)
y(1,1)=p(1)*q(1)-p(2)*q(2)-p(3)*q(3)-p(4)*q(4);
y(2,1)=p(1)*q(2)+p(2)*q(1)+p(3)*q(4)-p(4)*q(3);
y(3,1)=p(1)*q(3)-p(2)*q(4)+p(3)*q(1)+p(4)*q(2);
y(4,1)=p(1)*q(4)+p(2)*q(3)-p(3)*q(2)+p(4)*q(1);
end
