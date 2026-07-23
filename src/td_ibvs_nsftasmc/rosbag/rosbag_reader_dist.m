experimentbag = rosbag('/home/armando/Documents/visual_servoing_ws/src/td_ibvs_nsftasmc/rosbag/2023-10-02-10-03-32.bag');
%% select reference to assign a experiment starting time
uav_pos = select(experimentbag, "Topic", '/quad_position');
uav_pos_x_ts = timeseries(uav_pos,'X');
end_time0 = uav_pos_x_ts.get.TimeInfo.End;
uav_pos_x_ts = delsample(uav_pos_x_ts, 'Value', end_time0);
start_time = uav_pos_x_ts.get.TimeInfo.Start;
end_time = uav_pos_x_ts.get.TimeInfo.End;
test_time = end_time - start_time;
%%
uav_vel = select(experimentbag, "Topic", '/quad_velocity');
uav_vel_x_ts = timeseries(uav_vel, 'X');
uav_vel_x_ts = delsample(uav_vel_x_ts, 'Value', end_time0);
uav_vel_x_data = uav_vel_x_ts.get.Data;
t = uav_vel_x_ts.get.Time - start_time;
tx = t;

uav_vel_y_ts = timeseries(uav_vel, 'Y');
uav_vel_y_ts = delsample(uav_vel_y_ts, 'Value', end_time0);
uav_vel_y_data = uav_vel_y_ts.get.Data;
t = uav_vel_y_ts.get.Time - start_time;
ty = t;

uav_vel_z_ts = timeseries(uav_vel, 'Z');
uav_vel_z_ts = delsample(uav_vel_z_ts, 'Value', end_time0);
uav_vel_z_data = uav_vel_z_ts.get.Data;
t = uav_vel_z_ts.get.Time - start_time;
tz = t;
%%
uav_att = select(experimentbag, "Topic", '/quad_attitude');
uav_att_roll_ts = timeseries(uav_att,'X');
uav_att_roll_ts = delsample(uav_att_roll_ts, 'Value', end_time0);
t = uav_att_roll_ts.get.Time - start_time;
tRoll = t;
roll_data = uav_att_roll_ts.Data;

uav_att_pitch_ts = timeseries(uav_att,'Y');
uav_att_pitch_ts = delsample(uav_att_pitch_ts, 'Value', end_time0);
t = uav_att_pitch_ts.get.Time - start_time;
tPitch = t;
pitch_data = uav_att_pitch_ts.Data;

uav_att_yaw_ts = timeseries(uav_att,'Z');
uav_att_yaw_ts = delsample(uav_att_yaw_ts, 'Value', end_time0);
yaw_data = uav_att_yaw_ts.Data;
yaw_data = wrapToPi(yaw_data);
t = uav_att_yaw_ts.get.Time - start_time;
tYaw = t;
%%
% des_pos = select(experimentbag, "Topic", '/desired_position');
% 
% z_des_ts = timeseries(des_pos, 'Z');
% z_des_ts = delsample(z_des_ts, 'Value', end_time0);
% z_des_data = z_des_ts.get.Data;
% t = z_des_ts.get.Time - start_time;
% tz_des = t;
%%
x_vel = [tx uav_vel_x_data];
y_vel = [ty uav_vel_y_data];
z_vel = [ty uav_vel_z_data];
roll = [tRoll roll_data];
pitch = [tPitch pitch_data];
yaw = [tYaw yaw_data];
%z_des = [tz_des z_des_data];

