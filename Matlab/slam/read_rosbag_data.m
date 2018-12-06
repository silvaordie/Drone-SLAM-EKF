function [d1, d2, d4] = read_rosbag_data()
%% read rosbag data
bag = rosbag('2018-11-26-17-09-28.bag');
linearx = select(bag,'Topic','/imu/linearx');
lineary = select(bag,'Topic','/imu/lineary');
% linearz = select(bag,'Topic','/imu/linearz');
anchors = select(bag,'Topic','/front_uwb_driver_node/front_tag_readings');
%anchors= select(bag,'Topic','/front_uwb_visualization_node/marker_anchors');
d1 = readMessages(linearx);
d1{1};
d2 = readMessages(lineary);
d2{1};
% d3 = readMessages(linearz);
% d3{1};
d4 = readMessages(anchors);
d4{1};
end