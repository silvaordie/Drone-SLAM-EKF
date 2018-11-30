function [lm, e]=trilat(v, z,dr)

    %Preenche A e b
    for k=1:1:(length(z)-1)        
       A(k,:)=(2*(v(:,1)-v(:,k+1)))';
       b(k,1)=z(1,1) - z(1,k+1) - norm(v(:,1))^2 + norm(v(:,k+1))^2;       
    end
    
    %Ax-b=0, x=A^-1*b
    lm=-A\b;
    if(rank(A)==1)
        lm(2,1)=sqrt(dr-lm(1,1)^2);
    end
    %Calcula o maior desvio
    e=0;
    for k=1:1:length((z)-1)        
       d=norm(v(:,k)-lm) ;
       if(abs(d-sqrt(z(1,k)))>e)
        e=abs(d-sqrt(z(1,k)));
       end       
    end
end