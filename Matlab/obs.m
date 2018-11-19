function h=obs(x, L)

    first= [x-L(1,:) ; x-L(2,:) ; x-L(3,:) ];
    h = [ first(1,1)^2 + first(1,2)^2 ; first(2,1)^2 + first(2,2)^2 ; first(3,1)^2 + first(3,2)^2 ];
    h= h + rand(3,1)*0.2;
end