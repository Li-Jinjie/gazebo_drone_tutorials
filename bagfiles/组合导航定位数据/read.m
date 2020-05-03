% clc;clear;
% tag_path='tags_pose.bag';
% ground_truth_path = 'ground_truth.bag';
% imu_path = 'filtered_odom.bag';
% nav_path = 'filtered_map.bag';
% 
% tag_bag = rosbag(tag_path);
% gr_bag = rosbag(ground_truth_path);
% imu_bag = rosbag(imu_path);
% nav_bag = rosbag(nav_path); 
% 
% st = max([tag_bag.StartTime,gr_bag.StartTime,imu_bag.StartTime,nav_bag.StartTime]);
% 
% tag_bagS = select(tag_bag,'Time',...
%     [tag_bag.StartTime tag_bag.StartTime + 60],'Topic','/uav1/tags_pose');
% gr_bagS = select(gr_bag,'Time',...
%     [st st + 60],'Topic','/uav1/ground_truth/state');
% imu_bagS = select(imu_bag,'Time',...
%     [st st + 60],'Topic','/uav1/odometry/filtered_odom');
% nav_bagS = select(nav_bag,'Time',...
%     [st st + 60],'Topic','/uav1/odometry/filtered_map');
% 
% tag_msg = readMessages(tag_bagS);
% tag_x = cellfun(@(m) m.Pose.Pose.Position.X,tag_msg);
% tag_y = cellfun(@(m) m.Pose.Pose.Position.Y,tag_msg);
% tag_z = cellfun(@(m) m.Pose.Pose.Position.Z,tag_msg);
% 
% gr_msg = readMessages(gr_bagS);
% gr_x = cellfun(@(m) m.Pose.Pose.Position.X,gr_msg);
% gr_y = cellfun(@(m) m.Pose.Pose.Position.Y,gr_msg);
% gr_z = cellfun(@(m) m.Pose.Pose.Position.Z,gr_msg);
% 
% imu_msg = readMessages(imu_bagS);
% imu_x = cellfun(@(m) m.Pose.Pose.Position.X,imu_msg);
% imu_y = cellfun(@(m) m.Pose.Pose.Position.Y,imu_msg);
% imu_z = cellfun(@(m) m.Pose.Pose.Position.Z,imu_msg);
% 
% nav_msg = readMessages(nav_bagS);
% nav_x = cellfun(@(m) m.Pose.Pose.Position.X,nav_msg);
% nav_y = cellfun(@(m) m.Pose.Pose.Position.Y,nav_msg);
% nav_z = cellfun(@(m) m.Pose.Pose.Position.Z,nav_msg);

figure(1)
plot3(tag_x,tag_y,tag_z)
hold on;
plot3(gr_x,gr_y,gr_z)
hold on;
plot3(imu_x,imu_y,imu_z)
hold on;
plot3(nav_x,nav_y,nav_z)
grid on;
title('空间定位精度图');
xlabel('X/meters');
ylabel('Y/meters');
zlabel('Z/meters');
legend('二维码定位结果','系统真值','仅惯导解算结果','组合导航结果');

figure(2)

time_tag = linspace(0,60,length(tag_x))';
time_gr = linspace(0,60,length(gr_x))';

subplot(2,2,1)
plot(tag_x,tag_y)
hold on;
plot(gr_x,gr_y)
grid on;
title('水平定位精度');
xlabel('X/meters');
ylabel('Y/meters');
legend('二维码定位结果','系统真值');

subplot(2,2,2)
plot(time_tag,tag_x)
hold on;
plot(time_gr,gr_x)
grid on;
title('X方向定位精度');
xlabel('time/s');
ylabel('X/meters');
legend('二维码定位结果','系统真值');

subplot(2,2,3)
plot(time_tag,tag_y)
hold on;
plot(time_gr,gr_y)
grid on;
title('Y方向定位精度');
xlabel('time/s');
ylabel('Y/meters');
legend('二维码定位结果','系统真值');

subplot(2,2,4)
plot(time_tag,tag_z)
hold on;
plot(time_gr,gr_z)
grid on;
title('Z方向定位精度');
xlabel('time/s');
ylabel('Z/meters');
legend('二维码定位结果','系统真值');


