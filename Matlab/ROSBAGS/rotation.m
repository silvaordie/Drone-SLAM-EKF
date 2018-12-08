
function r=rotation(ang)

    r=zeros(3);
    a=ang(1,1);
    b=ang(2,1);
    g=ang(3,1);
    
    r(1,1)=cos(g)*cos(b);
    r(1,2)=cos(g)*sin(b)*sin(a)-sin(g)*cos(a);
    r(1,3)=cos(g)*sin(b)*cos(a)+sin(g)*sin(a);
    
    r(2,1)=sin(g)*cos(b);
    r(2,2)=sin(g)*sin(b)*sin(a)+cos(g)*cos(a);
    r(2,3)= sin(g)*sin(b)*cos(g)-cos(g)*sin(a);
    
    r(3,1)=-sin(b);
    r(3,2)=cos(b)*sin(a);
    r(3,3)=cos(b)*cos(a);
end