
clear all;
close all;
clc;

 
%bag = rosbag('mavros_2021-11-21-13-37-52.bag');
%bag = rosbag('mavros_zed_2021-11-21-13-56-13.bag');
%bag = rosbag('mavros_2021-12-04-17-16-30.bag');
 %bag = rosbag('mavros_2021-12-13-09-56-14.bag');
 bag = rosbag('2023-01-13-21-03-06.bag');
 

state_select = select(bag, 'Time',[bag.StartTime bag.EndTime], 'Topic', '/mavros/local_position/odom');% {'iiwa_msgs/JointPosition'}
stateMsgs = readMessages(state_select);

%vel_select = select(bag, 'Time',[bag.StartTime bag.EndTime], 'Topic', '/mavros/local_position/velocity_body');% {'iiwa_msgs/JointPosition'}
%vel_Msgs = readMessages(vel_state);

%vel_state = timeseries(vel_select, 'Twist.Linear.X','Twist.Angular.Z');% uppercase

cmd_select = select(bag, 'Time',[bag.StartTime bag.EndTime], 'Topic', '/position_cmd');% {'iiwa_msgs/JointPosition'}
cmdMsgs = readMessages(cmd_select);

att_select = select(bag, 'Time',[bag.StartTime bag.EndTime], 'Topic', '/debugPx4ctrl');% {'iiwa_msgs/JointPosition'}
attMsgs = readMessages(att_select);

rate_select = select(bag, 'Time',[bag.StartTime bag.EndTime], 'Topic', '/mavros/setpoint_raw/attitude');% {'iiwa_msgs/JointPosition'}
rateMsgs = readMessages(rate_select);

%% local position
%poseMsgs = stateMsgs{:,1}.Pose;
ts_state = timeseries(state_select, 'Pose.Pose.Position.X','Pose.Pose.Position.Y','Pose.Pose.Position.Z',...
    'Pose.Pose.Orientation.W','Pose.Pose.Orientation.X','Pose.Pose.Orientation.Y','Pose.Pose.Orientation.Z',...
    'Twist.Twist.Linear.X','Twist.Twist.Linear.Y','Twist.Twist.Linear.Z',...
    'Twist.Twist.Angular.X','Twist.Twist.Angular.Y','Twist.Twist.Angular.Z');% uppercase
t = ts_state.Time; 
x  = ts_state.Data(:,1);
y = ts_state.Data(:,2);
z = ts_state.Data(:,3);
vel = ts_state.Data(:,8:10);
orient = ts_state.Data(:,4:7);
ang_vel = ts_state.Data(:,11:13);
%plot(x,y)
% plot state in relative time.
t_relative = ts_state.Time - ts_state.Time(1,1); 
[yaw,pitch,roll] = quat2angle(orient);
uav_line_vel = ts_state.Data(:,8:10);
uav_ang_vel = ts_state.Data(:,11:13);

%% cmd mesg
cmd_state = timeseries(cmd_select, 'Position.X', 'Position.Y', 'Position.Z', 'Velocity.X', 'Velocity.Y', 'Velocity.Z',...
    'Acceleration.X', 'Acceleration.Y', 'Acceleration.Z', 'Jerk.X','Jerk.Y', 'Jerk.Z', 'Yaw', 'YawDot');
traj_t = cmd_state.Time;
traj_t_relative = cmd_state.Time - cmd_state.Time(1,1);
traj_pos = cmd_state.Data(:,1:3);
traj_vel = cmd_state.Data(:,4:6);
traj_acc = cmd_state.Data(:,7:9);
traj_jerk = cmd_state.Data(:,10:12);
traj_yaw = cmd_state.Data(:,13);
traj_yaw = cmd_state.Data(:,14);

%% att mesg
att_state = timeseries(att_select,'DesQW','DesQX', 'DesQY','DesQZ');
t_att = att_state.Time;
des_att_quat = att_state.Data(:,1:4);

[des_yaw, des_pitch, des_roll] = quat2angle(des_att_quat);

rate_state = timeseries(rate_select,"BodyRate.X","BodyRate.Y", "BodyRate.Z","Thrust");

t_rate = rate_state.Time;
des_rate = rate_state.Data(:,1:3);
des_thr = rate_state.Data(:,4);

figure(1)
subplot(3,1,1)
plot(t,x,'-r','linewidth',2);
hold on
plot(traj_t,traj_pos(:,1),'-b','linewidth',2);

subplot(3,1,2)
plot(t,y,'-r','linewidth',2);
hold on
plot(traj_t,traj_pos(:,2),'-b','linewidth',2);


subplot(3,1,3)
plot(t,z,'-r','linewidth',2);
hold on
plot(traj_t,traj_pos(:,3),'-b','linewidth',2);

figure(2)
subplot(3,1,1)
plot(t,vel(:,1),'-r','linewidth',2);
hold on
plot(traj_t,traj_vel(:,1),'-b','linewidth',2);

subplot(3,1,2)
plot(t,vel(:,2),'-r','linewidth',2);
hold on
plot(traj_t,traj_vel(:,2),'-b','linewidth',2);


subplot(3,1,3)
plot(t,vel(:,3),'-r','linewidth',2);
hold on
plot(traj_t,traj_vel(:,3),'-b','linewidth',2);

figure(3)
plot(x,y,'-r','linewidth',2);
hold on
plot(traj_pos(:,1), traj_pos(:,2), '-b','linewidth',2);

figure(4)
subplot(2,1,1);
plot(t_att, rad2deg(des_roll),'-b','linewidth',2);
hold on 
plot(t, rad2deg(roll),'-r','linewidth',2);

subplot(2,1,2)
plot(t_att, rad2deg(des_pitch), '-b','linewidth',2);
hold on
plot(t, rad2deg(pitch), '-r','linewidth',2);

