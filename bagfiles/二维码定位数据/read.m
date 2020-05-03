% clc;clear;
% tag_filepath='tags_pose.bag';
% ground_truth_filepath = 'ground_truth.bag';
% tag_bag=rosbag(tag_filepath);
% g_bag=rosbag(ground_truth_filepath);
% 
% st = tag_bag.StartTime;
% 
% tag_bagS = select(tag_bag,'Time',...
%     [tag_bag.StartTime tag_bag.StartTime + 60],'Topic','/uav1/tags_pose');
% gr_bagS = select(g_bag,'Time',...
%     [st st + 60],'Topic','/uav1/ground_truth/state');
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

figure(1)
plot3(tag_x,tag_y,tag_z)
hold on;
plot3(gr_x,gr_y,gr_z)
grid on;
title('空间定位精度图');
xlabel('X/meters');
ylabel('Y/meters');
zlabel('Z/meters');
legend('二维码定位结果','系统真值');

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


