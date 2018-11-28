clear 

V=[1; 2];
x(1:2,1)=[0;0];
x(1:2,1)=V;
LS=[ 1 2 1 ; -1 2 1 ];

for k=2:1:50
    
   x(1:2,k)=x(1:2,k-1)+V;
   dr(1,k)=norm(x(:,k)-LS(:,1));
   dr(2,k)=norm(x(:,k)-LS(:,2));
   dr(3,k)=norm(x(:,k)-LS(:,3));
  
end

cvx_begin quiet

variable v(2,1);
variable L(2,3);
xp(1:2,1)=[0;0];
err=0;
for k=2:1:50
   d(1,k)=norm(xp-L(:,1));
   d(2,k)=norm(xp-L(:,2));
   d(3,k)=norm(xp-L(:,3));
   
   err(1)=err(1) + square_pos(norm(square_pos(d(1,k))-square_pos(dr(1,k))));
   err(2)=err(2) + square_pos(norm(square_pos(d(2,k))-square_pos(dr(2,k))));
   err(3)=err(3) + square_pos(norm(square_pos(d(3,k))-square_pos(dr(3,k))));
end

min( sum(err) );

for k=2:1:50
      xp(1:2,k)  == xp(1:2,k-1)+v;  
end

xp(1:2,1)=[0 0]
cvx_end;