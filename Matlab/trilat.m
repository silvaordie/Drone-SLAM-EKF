function lm=trilat(v, z)
    cvx_begin quiet
    variable x(2);
    
    
    %som =  power( abs(norm(x(1,1:2)-w(:,1)')-r) , 2 ) ;
    %som = ((x(tau(1),1)-w(1,1))^2 +  (x(tau(1),2)-w(2,1))^2) + r^2 + 2*norm(x(1,1:2)-w(:,1)')*r;
    som = 0;
    for k=1:2:20
      % som = ((x(tau(k),1)-w(1,k))^2 +  (x(tau(k),2)-w(2,k))^2) + r^2 + 2*norm(x(k,1:2)-w(:,k)')*r + som;
      som =  square_pos( square_pos(norm( x-v(:,k))) - square_pos(z(1,k))) + som ;
    end
    
    minimize( som )
    
    cvx_end;
    
    lm=x;
end