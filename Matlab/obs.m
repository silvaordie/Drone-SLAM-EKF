function h=obs(x, L)

    first= [x-L(1,:) ; x-L(2,:) ; x-L(3,:) ];
    h = [ first(1,1)^2 + first(1,2)^2 ; first(2,1)^2 + first(2,2)^2 ; first(3,1)^2 + first(3,2)^2 ];
    for k=1:1:length(h)
       if(h(k,1)<1000000000000000)
           h(k,1)=h(k,1)+rand(1,1)*0.2;
       else
           h(k,1)=10000000;   
       end
    end
end