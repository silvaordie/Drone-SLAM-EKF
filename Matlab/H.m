function h=H(x, LM)
    
    h=zeros(LM,(6+2*LM));
    
    for k=0:1:LM-1
        h(k+1,1)= 2*x(1)- 2*x(7+2*k);
        h(k+1, 2)= 2*x(2)- 2*x(7+2*k+1);
        h(k+1, 7+2*k)=2*x(7+2*k)- 2*x(1);
        h(k+1, 7+2*k+1)= 2*x(7+2*k+1)- 2*x(2);
    end
    
end