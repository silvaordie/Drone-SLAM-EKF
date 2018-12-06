clear;

[d, v_ang, var]=read_rosbag_data();
d=abs(d);
THRESHOLD=0.15;
k=2;

while(abs(d(1,k))>THRESHOLD && abs(d(2,k))>THRESHOLD  && abs(d(3,k))>THRESHOLD )
    k=k+1;
end
tinicial=k-1;

if(d(1,k)<d(2,k) && d(1,k)<d(3,k))
    start=1;
else if(d(2,k)<d(3,k))
        start=2;
    else
        start=3;
    end
end

switch start
    case 1
        landmarks=[2; 3];
    case 2 
        landmarks=[1;3];
    case 3
        landmarks=[1;2];
end

while(abs(d(landmarks(1,1),k)) >THRESHOLD && abs(d(landmarks(2,1),k)) >THRESHOLD)
    trajectory(1,k-tinicial)=-d(start,k);
    trajectory(2,k-tinicial)=0;
    k=k+1;
end   

if(abs(d(landmarks(1,1),k))>abs(d(landmarks(2,1),k)))
    finish=landmarks(2,1);
    estimate=landmarks(1,1);
else
    finish=landmarks(1,1);
    estimate=landmarks(2,1);
end

tfinal=k-1;

[lm,e]=trilat(trajectory ,d(estimate,tinicial:tfinal-1), d(estimate,tfinal));

 LM(:,start)=[trajectory(1,1),0];
 LM(:,finish)=[0,0];
 LM(:,estimate)=lm;