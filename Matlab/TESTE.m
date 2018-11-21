load matlab.mat;

v=x(1:2, :);
z=z(:, :);
N=length(z);
for k=1:1:(length(z)-1)
        
       A(k,:)=(2*(v(:,k)-v(:,N)))';
       b(k,1)=-z(1,k)^2 + z(1,N)^2 + norm(v(:,k))^2 - norm(v(:,N))^2;
        
end

lm=A\b