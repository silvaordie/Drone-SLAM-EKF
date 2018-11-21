function lm=trilat(v, z)


    for k=1:1:(length(z)-1)
        
       A(k,:)=(2*(v(:,1)-v(:,k+1)))';
       b(k)=z(1,1)^2 - z(1,k+1)^2 + norm(v(:,1))^2 - norm(v(:,k+1))^2;
        
    end
    
    lm=A\b
    
end