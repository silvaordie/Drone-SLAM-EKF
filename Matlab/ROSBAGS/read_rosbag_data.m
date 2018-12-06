function [distancia, variancia, v_ang] =  read_rosbag_data()
%% read rosbag data
%fid1 = fopen('topics.txt','r');
fid1 = fopen('tag.txt','r');
k = 1;
while ~feof(fid1)
    line = fgets(fid1); 
    A = sscanf(line,'%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s'); %# sscanf can read only numeric data :(
    A = strsplit(A,',');
    
    dist1 = A{7}; dist2 = A{10}; dist3 = A{13};
    v1 = A{8}; v2 = A{11}; v3 = A{14};

    d1 = str2num(dist1); d2 = str2num(dist2); d3 = str2num(dist3);
    var1 = str2num(v1); var2 = str2num(v2); var3 = str2num(v3);
    
    distancia(1,k) = d1; distancia(2,k) = d2; distancia(3,k) = d3;
    variancia(1,k) = var1; variancia(2,k) = var2; variancia(3,k) = var3;
    k = k+1;
end
fclose(fid1);
fid2 = fopen('angularx.txt', 'r'); % angular velocities 1
k = 1;
while ~feof(fid2)
    line = fgets(fid2);
    A = sscanf(line,'%s,%s');
    A = strsplit(A,',');
    v_ang(1,k) = str2num(A{2});
    k = k+1;
end
fclose(fid2);
fid3 = fopen('angulary.txt', 'r'); % angular velocities 2
k = 1;
while ~feof(fid3)
    line = fgets(fid3);
    A = sscanf(line,'%s,%s');
    A = strsplit(A,',');
    v_ang(2,k) = str2num(A{2});
    k = k+1;
end
fclose(fid3);
fid4 = fopen('angularz.txt', 'r'); % angular velocities 2
k = 1;
while ~feof(fid4)
    line = fgets(fid4);
    A = sscanf(line,'%s,%s');
    A = strsplit(A,',');
    v_ang(3,k) = str2num(A{2});
    k = k+1;
end
%d1 = d1.^2; d2 = d2.^2; d3 = d3.^2;
fclose(fid4);
end