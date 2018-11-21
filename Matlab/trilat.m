function [lm, e]=trilat(v, z)


    for k=1:1:(length(z)-1)
        
       A(k,:)=(2*(v(:,1)-v(:,k+1)))';
       b(k,1)=z(1,1)^2 - z(1,k+1)^2 + norm(v(:,1))^2 - norm(v(:,k+1))^2;
        
    end
      
    lm=A\b;
    
    e=0;
    for k=1:1:length((z)-1)
        
       d=norm(v(:,k)-lm) ;
       if(abs(d-z(1,k))>e)
        e=abs(d-z(1,k));
       end
       
    end
end