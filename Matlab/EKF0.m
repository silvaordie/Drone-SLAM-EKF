clear 

V=[0.1; 0.2];
x(1:2,1)=[0;0];
LS=[ 6 2 10 ; -1 5 6 ];

for k=2:1:50
    
   x(1:2,k)=x(1:2,k-1)+V;
   first= [(x(:,k)-LS(:,1))' ; (x(:,k)-LS(:,2))' ; (x(:,k)-LS(:,3))' ];
   dr(:,k) = [ first(1,1)^2 + first(1,2)^2 ; first(2,1)^2 + first(2,2)^2 ; first(3,1)^2 + first(3,2)^2 ];
    
  
end

for k=1:1:50
    
   xd(1:2,k)=x(1:2,k)+abs(0.1*rand(1,1)*V);
   first= [(xd(:,k)-LS(:,1))' ; (xd(:,k)-LS(:,2))' ; (xd(:,k)-LS(:,3))' ];
   d(:,k) = [ first(1,1)^2 + first(1,2)^2 ; first(2,1)^2 + first(2,2)^2 ; first(3,1)^2 + first(3,2)^2 ];
    
end
v(1,:)=x(2,:);
v(2,:)=x(1,:);
z=d(1,:);


[lm, e]=trilat(x, d)
