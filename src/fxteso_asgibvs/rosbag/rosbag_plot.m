close all;
%% declare name of the bag
experimentbag = rosbag('circle.bag');

%% select reference to assign a experiment starting time
uav_pos = select(experimentbag, "Topic", '/quad_position');
uav_pos_x_ts = timeseries(uav_pos,'X');
end_time0 = uav_pos_x_ts.get.TimeInfo.End;
uav_pos_x_ts = delsample(uav_pos_x_ts, 'Value', end_time0);
start_time = uav_pos_x_ts.get.TimeInfo.Start;
end_time = uav_pos_x_ts.get.TimeInfo.End;
test_time = end_time - start_time;

%% x and desired x plot
%x-UAV
t = uav_pos_x_ts.get.Time - start_time;
x_data = uav_pos_x_ts.get.Data;
figure
plot(t,x_data,'r')
hold on
%desired x
tgt_pos = select(experimentbag, "Topic", '/tgt_position');
tgt_x_ts = timeseries(tgt_pos, 'X');
%tgt_x_ts =  getsampleusingtime(tgt_x_ts, start_time, end_time);
t = tgt_x_ts.get.Time - start_time;
tgt_x_data = tgt_x_ts.get.Data;
plot(t,tgt_x_data,'r--')
legend('$x$','$x_{d}$','Interpreter', 'latex','Box','off','FontSize',18);
xlabel('Time [s]', 'Interpreter', 'latex','FontSize',20) 
ylabel('Position [m]', 'Interpreter', 'latex','FontSize',20)
xlim([0 test_time])
%% y and desired y plot
%y
uav_pos_y_ts = timeseries(uav_pos, 'Y');
uav_pos_y_ts = delsample(uav_pos_y_ts, 'Value', end_time0);
t = uav_pos_y_ts.get.Time - start_time;
t = t - t(1);
y_data = uav_pos_y_ts.Data;
figure
plot(t,y_data,'b')
hold on
%desired y
tgt_y_ts = timeseries(tgt_pos, 'Y');
tgt_y_ts = delsample(tgt_y_ts, 'Value', end_time0);
t = tgt_y_ts.get.Time - start_time;
tgt_y_data = tgt_y_ts.get.Data;
plot(t,tgt_y_data,'b--')
legend('$y$','$y_{d}$','Interpreter', 'latex','Box','off','FontSize',18);
xlabel('Time [s]', 'Interpreter', 'latex','FontSize',20) 
ylabel('Position [m]', 'Interpreter', 'latex','FontSize',20)
xlim([0 test_time])
%% z and desired z plot
%z
uav_pos_z_ts = timeseries(uav_pos, 'Z');
uav_pos_z_ts = delsample(uav_pos_z_ts, 'Value', end_time0);
t = uav_pos_z_ts.get.Time - start_time;
z_data = uav_pos_z_ts.Data;
figure
plot(t,z_data,'Color',[0.0314 0.353 0]);
hold on
%desired z
tgt_z_ts = timeseries(tgt_pos, 'Z');
tgt_z_ts = delsample(tgt_z_ts, 'Value', end_time0);
t = tgt_z_ts.get.Time - start_time;
tgt_z_data = -2.5+tgt_z_ts.get.Data;
plot(t,tgt_z_data,'b--')
% z_desired = select(experimentbag, "Topic", '/z_des');
% z_des_ts = timeseries(z_desired, 'Data');
% z_des_ts = delsample(z_des_ts, 'Value', end_time0);
% t = z_des_ts.get.Time - start_time;
% z_des_data = z_des_ts.get.Data;
% plot(t,z_des_data,'Color',[0.0314 0.353 0],'LineStyle','--')
hold off
legend('$z$','$z_{d}$','Interpreter', 'latex','Box','off','FontSize',18);
xlabel('Time [s]', 'Interpreter', 'latex','FontSize',20) 
ylabel('Position [m]', 'Interpreter', 'latex','FontSize',20)
xlim([0 test_time])
%% XY plot
figure
plot(y_data,x_data,'r')
hold on
%Desired Traj
plot(tgt_y_data,tgt_x_data,'k--')
legend('FAH', 'Target', 'Interpreter', 'latex','Box','off','FontSize',18)
xlabel('East [m]', 'Interpreter', 'latex','FontSize',20) 
ylabel('North [m]', 'Interpreter', 'latex','FontSize',20)
xlim([-12 12])

%% UAV Roll and Pitch
uav_att = select(experimentbag, "Topic", '/quad_attitude');
uav_att_roll_ts = timeseries(uav_att,'X');
uav_att_roll_ts = delsample(uav_att_roll_ts, 'Value', end_time0);
t = uav_att_roll_ts.get.Time - start_time;
roll_data = uav_att_roll_ts.Data;
figure;
plot(t,roll_data,'r')
hold on;
uav_att_pitch_ts = timeseries(uav_att,'Y');
uav_att_pitch_ts = delsample(uav_att_pitch_ts, 'Value', end_time0);
t = uav_att_pitch_ts.get.Time - start_time;
pitch_data = uav_att_pitch_ts.Data;
plot(t,pitch_data,'b')
legend('$\phi$','$\theta$','Interpreter', 'latex','Box','off','FontSize',18);
xlabel('Time [s]', 'Interpreter', 'latex','FontSize',20) 
ylabel('Rotation [rad]', 'Interpreter', 'latex','FontSize',20)
xlim([0 test_time])

%% yaw and desired yaw plot
% UAV Yaw
uav_att_yaw_ts = timeseries(uav_att,'Z');
uav_att_yaw_ts = delsample(uav_att_yaw_ts, 'Value', end_time0);
t = uav_att_yaw_ts.get.Time - start_time;
yaw_data = uav_att_yaw_ts.Data;
yaw_data = wrapToPi(yaw_data);
figure;
plot(t,yaw_data,'r')
hold on
% Target yaw
tgt_yaw = select(experimentbag, "Topic", '/tgt_yaw');
tgt_yaw_ts = timeseries(tgt_yaw,'Data');
tgt_yaw_ts = delsample(tgt_yaw_ts, 'Value', end_time0);
t = tgt_yaw_ts.get.Time - start_time;
tgt_yaw_data = tgt_yaw_ts.Data;
tgt_yaw_data = wrapToPi(tgt_yaw_data);
plot(t,tgt_yaw_data,'k--')
legend('$\psi$','$\psi_d$','Interpreter', 'latex','Box','off','FontSize',18);
xlabel('Time [s]', 'Interpreter', 'latex','FontSize',20) 
ylabel('Rotation [rad]', 'Interpreter', 'latex','FontSize',20)
xlim([0 test_time])

%% IBVS Adaptive Gains
% IBVS_AG_x
ibvs_adap = select(experimentbag, "Topic", '/adaptive_gain');
ibvs_adap_x_ts = timeseries(ibvs_adap, 'X');
ibvs_adap_x_ts = delsample(ibvs_adap_x_ts, 'Value', end_time0);
t = ibvs_adap_x_ts.get.Time - start_time;
ibvs_adap_x_data = ibvs_adap_x_ts.get.Data;
figure
plot(t,ibvs_adap_x_data,'r')
hold on
% IBVS_AG_x
ibvs_adap_y_ts = timeseries(ibvs_adap, 'Y');
ibvs_adap_y_ts = delsample(ibvs_adap_y_ts, 'Value', end_time0);
t = ibvs_adap_y_ts.get.Time - start_time;
ibvs_adap_y_data = ibvs_adap_y_ts.get.Data;
plot(t,ibvs_adap_y_data,'b')
% IBVS_AG_z
ibvs_adap_z_ts = timeseries(ibvs_adap, 'Z');
ibvs_adap_z_ts = delsample(ibvs_adap_z_ts, 'Value', end_time0);
t = ibvs_adap_z_ts.get.Time - start_time;
ibvs_adap_z_data = ibvs_adap_z_ts.get.Data;
plot(t,ibvs_adap_z_data,'Color',[0.0314 0.353 0])
% IBVS_AG_yaw
ibvs_adap_yaw_ts = timeseries(ibvs_adap, 'W');
ibvs_adap_yaw_ts = delsample(ibvs_adap_yaw_ts, 'Value', end_time0);
t = ibvs_adap_yaw_ts.get.Time - start_time;
ibvs_adap_yaw_data = ibvs_adap_yaw_ts.get.Data;
plot(t,ibvs_adap_yaw_data,'Color',[176/255 88/255 0])
legend('$x$','$y$', '$z$', '$\psi$', 'Interpreter', 'latex','Box','off','FontSize',18)
xlabel('Time [s]', 'Interpreter', 'latex','FontSize',20) 
ylabel('$K_1$', 'Interpreter', 'latex','FontSize',20) 
xlim([0 test_time])
% ylim([-35 45])
%% Image Features Estimates vs Measured
imFeat = select(experimentbag, "Topic", '/ImFeat_vector');
imFeatEst = select(experimentbag, "Topic", '/ImFeat_estimates_fxt');
%Image Feature x
ImFeat_x_ts = timeseries(imFeat, 'X');
ImFeat_x_ts = delsample(ImFeat_x_ts, 'Value', end_time0);
t = ImFeat_x_ts.get.Time - start_time;
ImFeat_x_data = ImFeat_x_ts.get.Data;
figure;
plot(t,ImFeat_x_data,'r--')
hold on;
ImFeatEst_x_ts = timeseries(imFeatEst, 'X');
ImFeatEst_x_ts = delsample(ImFeatEst_x_ts, 'Value', end_time0);
t = ImFeatEst_x_ts.get.Time - start_time;
ImFeatEst_x_data = ImFeatEst_x_ts.get.Data;
plot(t,ImFeatEst_x_data,'r')
legend('$\xi_x$','$\hat{\xi}_x$', 'Interpreter', 'latex','Box','off','FontSize',18)
xlabel('Time [s]', 'Interpreter', 'latex','FontSize',20) 
ylabel('Value', 'Interpreter', 'latex','FontSize',20) 
xlim([0 test_time])
%Image Feature y
ImFeat_y_ts = timeseries(imFeat, 'Y');
ImFeat_y_ts = delsample(ImFeat_y_ts, 'Value', end_time0);
t = ImFeat_y_ts.get.Time - start_time;
ImFeat_y_data = ImFeat_y_ts.get.Data;
figure;
plot(t,ImFeat_y_data,'b--')
hold on;
ImFeatEst_y_ts = timeseries(imFeatEst, 'Y');
ImFeatEst_y_ts = delsample(ImFeatEst_y_ts, 'Value', end_time0);
t = ImFeatEst_y_ts.get.Time - start_time;
ImFeatEst_y_data = ImFeatEst_y_ts.get.Data;
plot(t,ImFeatEst_y_data,'b')
legend('$\xi_y$','$\hat{\xi}_y$', 'Interpreter', 'latex','Box','off','FontSize',18)
xlabel('Time [s]', 'Interpreter', 'latex','FontSize',20) 
ylabel('Value', 'Interpreter', 'latex','FontSize',20) 
xlim([0 test_time])
%Image Feature z
ImFeat_z_ts = timeseries(imFeat, 'Z');
ImFeat_z_ts = delsample(ImFeat_z_ts, 'Value', end_time0);
t = ImFeat_z_ts.get.Time - start_time;
ImFeat_z_data = ImFeat_z_ts.get.Data;
figure;
plot(t,ImFeat_z_data,'Color',[0.0314 0.353 0],'LineStyle','--')
hold on
ImFeatEst_z_ts = timeseries(imFeatEst, 'Z');
ImFeatEst_z_ts = delsample(ImFeatEst_z_ts, 'Value', end_time0);
t = ImFeatEst_z_ts.get.Time - start_time;
ImFeatEst_z_data = ImFeatEst_z_ts.get.Data;
plot(t,ImFeatEst_z_data,'Color',[0.0314 0.353 0])
legend('$\xi_z$','$\hat{\xi}_z$', 'Interpreter', 'latex','Box','off','FontSize',18)
xlabel('Time [s]', 'Interpreter', 'latex','FontSize',20) 
ylabel('Value', 'Interpreter', 'latex','FontSize',20) 
xlim([0 test_time])
%Image Feature yaw
ImFeat_yaw_ts = timeseries(imFeat, 'W');
ImFeat_yaw_ts = delsample(ImFeat_yaw_ts, 'Value', end_time0);
t = ImFeat_yaw_ts.get.Time - start_time;
ImFeat_yaw_data = ImFeat_yaw_ts.get.Data;
figure;
plot(t,ImFeat_yaw_data,'Color',[176/255 88/255 0],'LineStyle','--')
hold on
ImFeatEst_yaw_ts = timeseries(imFeatEst, 'W');
ImFeatEst_yaw_ts = delsample(ImFeatEst_yaw_ts, 'Value', end_time0);
t = ImFeatEst_yaw_ts.get.Time - start_time;
ImFeatEst_yaw_data = ImFeatEst_yaw_ts.get.Data;
plot(t,ImFeatEst_yaw_data,'Color',[176/255 88/255 0])
legend('$\xi_\psi$','$\hat{\xi}_\psi$', 'Interpreter', 'latex','Box','off','FontSize',18)
xlabel('Time [s]', 'Interpreter', 'latex','FontSize',20) 
ylabel('Value', 'Interpreter', 'latex','FontSize',20) 
xlim([0 test_time])
%% Image Features Velocity Estimates vs Measured
imFeatEstDot = select(experimentbag, "Topic", '/ImFeat_dot_estimates_fxt');
%Image Feature dot x
ImFeatEstDot_x_ts = timeseries(imFeatEstDot, 'X');
ImFeatEstDot_x_ts = delsample(ImFeatEstDot_x_ts, 'Value', end_time0);
t = ImFeatEstDot_x_ts.get.Time - start_time;
ImFeatEstDot_x_data = ImFeatEstDot_x_ts.get.Data;
plot(t,ImFeatEstDot_x_data,'r')
hold on;
%Image Feature dot y
ImFeatEstDot_y_ts = timeseries(imFeatEstDot, 'Y');
ImFeatEstDot_y_ts = delsample(ImFeatEstDot_y_ts, 'Value', end_time0);
t = ImFeatEstDot_y_ts.get.Time - start_time;
ImFeatEstDot_y_data = ImFeatEstDot_y_ts.get.Data;
plot(t,ImFeatEstDot_y_data,'b')
%Image Feature dot z
ImFeatEstDot_z_ts = timeseries(imFeatEstDot, 'Z');
ImFeatEstDot_z_ts = delsample(ImFeatEstDot_z_ts, 'Value', end_time0);
t = ImFeatEstDot_z_ts.get.Time - start_time;
ImFeatEstDot_z_data = ImFeatEstDot_z_ts.get.Data;
plot(t,ImFeatEstDot_z_data,'Color',[0.0314 0.353 0])
%Image Feature dot yaw
ImFeatEstDot_yaw_ts = timeseries(imFeatEstDot, 'W');
ImFeatEstDot_yaw_ts = delsample(ImFeatEstDot_yaw_ts, 'Value', end_time0);
t = ImFeatEstDot_yaw_ts.get.Time - start_time;
ImFeatEstDot_yaw_data = ImFeatEstDot_yaw_ts.get.Data;
plot(t,ImFeatEstDot_yaw_data,'Color',[176/255 88/255 0])
legend('$\dot{\hat{\xi}}_x$','$\dot{\hat{\xi}}_y$','$\dot{\hat{\xi}}_z$','$\dot{\hat{\xi}}_\psi$', 'Interpreter', 'latex','Box','off','FontSize',18)
xlabel('Time [s]', 'Interpreter', 'latex','FontSize',20) 
ylabel('Value', 'Interpreter', 'latex','FontSize',20) 
xlim([0 test_time])
%% Estimation error
% EstErr = select(experimentbag, "Topic", '/estimation_error');
% %Estimation error x
% EstErr_x_ts = timeseries(EstErr, 'X');
% EstErr_x_ts = delsample(EstErr_x_ts, 'Value', end_time0);
% t = EstErr_x_ts.get.Time - start_time;
% EstErr_x_data = EstErr_x_ts.get.Data;
% figure;
% plot(t,EstErr_x_data,'r')
% hold on;
% %Estimation error y
% EstErr_y_ts = timeseries(EstErr, 'Y');
% EstErr_y_ts = delsample(EstErr_y_ts, 'Value', end_time0);
% t = EstErr_y_ts.get.Time - start_time;
% EstErr_y_data = EstErr_y_ts.get.Data;
% plot(t,EstErr_y_data,'b')
% %Estimation error z
% EstErr_z_ts = timeseries(EstErr, 'Z');
% EstErr_z_ts = delsample(EstErr_z_ts, 'Value', end_time0);
% t = EstErr_z_ts.get.Time - start_time;
% EstErr_z_data = EstErr_z_ts.get.Data;
% plot(t,EstErr_z_data,'Color',[0.0314 0.353 0])
% %Estimation error yaw
% EstErr_yaw_ts = timeseries(EstErr, 'W');
% EstErr_yaw_ts = delsample(EstErr_yaw_ts, 'Value', end_time0);
% t = EstErr_yaw_ts.get.Time - start_time;
% EstErr_yaw_data = EstErr_yaw_ts.get.Data;
% plot(t,EstErr_y_data,'Color',[176/255 88/255 0])
% legend('$\hat{e}_{\xi,x}$','$\hat{e}_{\xi,y}$','$\hat{e}_{\xi,z}$','$\hat{e}_{\xi,\psi}$','Interpreter', 'latex','Box','off','FontSize',18)
% xlabel('Time [s]', 'Interpreter', 'latex','FontSize',20) 
% ylabel('Estimation Error', 'Interpreter', 'latex','FontSize',20) 
% xlim([0 test_time])
% ylim([-0.08 0.08]);
%% UAV Thrust
thrust = select(experimentbag, "Topic", '/quad_thrust');
thrust_ts = timeseries(thrust, 'Data');
thrust_ts = delsample(thrust_ts, 'Value', end_time0);
t = thrust_ts.get.Time - start_time;
thrust_data = thrust_ts.get.Data;
figure;
plot(t,thrust_data,'r')
xlim([0 120]);
%% UAV Torques
Torques = select(experimentbag, "Topic", '/quad_torques');
%Torque x
Torques_x_ts = timeseries(Torques, 'X');
Torques_x_ts = delsample(Torques_x_ts, 'Value', end_time0);
t = Torques_x_ts.get.Time - start_time;
Torques_x_data = Torques_x_ts.get.Data;
figure;
plot(t,Torques_x_data,'r')
hold on
%Torque y
Torques_y_ts = timeseries(Torques, 'Y');
Torques_y_ts = delsample(Torques_y_ts, 'Value', end_time0);
t = Torques_y_ts.get.Time - start_time;
Torques_y_data = Torques_y_ts.get.Data;
plot(t,Torques_y_data,'b')
%Torque z
Torques_z_ts = timeseries(Torques, 'Z');
Torques_z_ts = delsample(Torques_z_ts, 'Value', end_time0);
t = Torques_z_ts.get.Time - start_time;
Torques_z_data = Torques_z_ts.get.Data;
plot(t,Torques_z_data,'Color',[0.0314 0.353 0])
legend('$\tau_x$','$\tau_y$','$\tau_z$','Interpreter', 'latex','Box','off','FontSize',18)
xlabel('Time [s]', 'Interpreter', 'latex','FontSize',20) 
ylabel('Torque [Nm]', 'Interpreter', 'latex','FontSize',20) 
xlim([0 test_time])
%% POI Evolution
% u-coordinates
u_pixel = select(experimentbag, "Topic", '/u_coordinates');
u_pixel1_ts = timeseries(u_pixel, 'X');
u_pixel1_ts = delsample(u_pixel1_ts, 'Value', end_time0);
u_pixel1_data = u_pixel1_ts.get.Data;

u_pixel2_ts = timeseries(u_pixel, 'Y');
u_pixel2_ts = delsample(u_pixel2_ts, 'Value', end_time0);
u_pixel2_data = u_pixel2_ts.get.Data;

u_pixel3_ts = timeseries(u_pixel, 'Z');
u_pixel3_ts = delsample(u_pixel3_ts, 'Value', end_time0);
u_pixel3_data = u_pixel3_ts.get.Data;

u_pixel4_ts = timeseries(u_pixel, 'W');
u_pixel4_ts = delsample(u_pixel4_ts, 'Value', end_time0);
u_pixel4_data = u_pixel4_ts.get.Data;

% n-coordinates
n_pixel = select(experimentbag, "Topic", '/n_coordinates');
n_pixel1_ts = timeseries(n_pixel, 'X');
n_pixel1_ts = delsample(n_pixel1_ts, 'Value', end_time0);
n_pixel1_data = n_pixel1_ts.get.Data;

n_pixel2_ts = timeseries(n_pixel, 'Y');
n_pixel2_ts = delsample(n_pixel2_ts, 'Value', end_time0);
n_pixel2_data = n_pixel2_ts.get.Data;

n_pixel3_ts = timeseries(n_pixel, 'Z');
n_pixel3_ts = delsample(n_pixel3_ts, 'Value', end_time0);
n_pixel3_data = n_pixel3_ts.get.Data;

n_pixel4_ts = timeseries(n_pixel, 'W');
n_pixel4_ts = delsample(n_pixel4_ts, 'Value', end_time0);
n_pixel4_data = n_pixel4_ts.get.Data;

u_pix = [u_pixel1_data u_pixel2_data u_pixel3_data u_pixel4_data];
n_pix = [n_pixel1_data n_pixel2_data n_pixel3_data n_pixel4_data];

ug = zeros(size(u_pix,1),1);
ng = zeros(size(n_pix,1),1);
for i=1:size(u_pix,1)
    ug(i) = (u_pixel1_data(i) + u_pixel2_data(i) + u_pixel3_data(i) + u_pixel4_data(i))/4;
    ng(i) = (n_pixel1_data(i) + n_pixel2_data(i) + n_pixel3_data(i) + n_pixel4_data(i))/4;
end
figure;
k = 2:1:size(ug,1);
for j = 1:4
    plot(u_pix(k,j),n_pix(k,j),'k--','LineWidth',0.25,'HandleVisibility','off');
    hold on;
    plot(u_pix(1,j),n_pix(1,j),'ro','MarkerFace','r', 'MarkerSize',10);
    plot(u_pix(end,j),n_pix(end,j),'d','MarkerFaceColor','b','MarkerEdgeColor','b','MarkerSize',10);
end
hold on;
%plot(ug(2:end-1), ng(2:end-1),'LineStyle','--', 'Color', [34/255 139/255 34/255] ,'LineWidth',0.25);
plot(ug(1),ng(1),'^','MarkerFaceColor','r','MarkerEdgeColor','r','MarkerSize',10);
plot(ug(end),ng(end),'v','MarkerFaceColor','b','MarkerEdgeColor','b','MarkerSize',10);
%title('Points of Interest', 'FontSize', 18,'FontWeight','bold','Interpreter','latex');
xlabel('u(pixel)', 'FontSize', 16,'FontWeight','bold','Interpreter','latex');
ylabel('n(pixel)', 'FontSize', 16,'FontWeight','bold','Interpreter','latex');
legend('Start','Final','$C_s$','$C_f$','$C_s$','$C_f$','$C_s$','$C_f$','$C_s$','$C_f$','FontSize', 16, 'Box','Off','FontWeight','Bold','Interpreter','latex'); 
xlim([-80 140])
%% Disturbances
dist = select(experimentbag, "Topic", '/disturbances');
%Disturbances x
disturbances_x_ts = timeseries(dist, 'X');
disturbances_x_ts = delsample(disturbances_x_ts, 'Value', end_time0);
t = disturbances_x_ts.get.Time - start_time;
disturbances_x_data = disturbances_x_ts.get.Data;
figure;
plot(t,disturbances_x_data,'r')
hold on
%Disturbances y
disturbances_y_ts = timeseries(dist, 'Y');
disturbances_y_ts = delsample(disturbances_y_ts, 'Value', end_time0);
t = disturbances_y_ts.get.Time - start_time;
disturbances_y_data = disturbances_y_ts.get.Data;
plot(t,disturbances_y_data,'b')
%Disturbances z
disturbances_z_ts = timeseries(dist, 'Z');
disturbances_z_ts = delsample(disturbances_z_ts, 'Value', end_time0);
t = disturbances_z_ts.get.Time - start_time;
disturbances_z_data = disturbances_z_ts.get.Data;
plot(t,disturbances_z_data,'Color',[0.0314 0.353 0])
legend('$\delta_x$','$\delta_y$','$\delta_z$','Interpreter', 'latex','Box','off','FontSize',18)
xlabel('Time [s]', 'Interpreter', 'latex','FontSize',20) 
ylabel('Drag force [N]', 'Interpreter', 'latex','FontSize',20) 
%xlim([10 40.5])

%% Visual Servoing Error
vs_error = select(experimentbag, "Topic", '/error_visual_servoing');
%Error x
vs_error_x_ts = timeseries(vs_error, 'X');
vs_error_x_ts = delsample(vs_error_x_ts, 'Value', end_time0);
t = vs_error_x_ts.get.Time - start_time;
vs_error_x_data = vs_error_x_ts.get.Data;
figure;
plot(t,vs_error_x_data,'r')
hold on
%Error y
vs_error_y_ts = timeseries(vs_error, 'Y');
vs_error_y_ts = delsample(vs_error_y_ts, 'Value', end_time0);
t = vs_error_y_ts.get.Time - start_time;
vs_error_y_data = vs_error_y_ts.get.Data;
plot(t,vs_error_y_data,'b')
%Error z
vs_error_z_ts = timeseries(vs_error, 'Z');
vs_error_z_ts = delsample(vs_error_z_ts, 'Value', end_time0);
t = vs_error_z_ts.get.Time - start_time;
vs_error_z_data = vs_error_z_ts.get.Data;
plot(t,vs_error_z_data,'Color',[0.0314 0.353 0])
%Error psi
vs_error_psi_ts = timeseries(vs_error, 'W');
vs_error_psi_ts = delsample(vs_error_psi_ts, 'Value', end_time0);
t = vs_error_psi_ts.get.Time - start_time;
vs_error_psi_data = vs_error_psi_ts.get.Data;
plot(t,vs_error_psi_data,'Color',[176/255 88/255 0])
legend('$e_{\xi,x}$','$e_{\xi,y}$', '$e_{\xi,z}$', '$e_{\xi,\psi}$','Interpreter', 'latex','Box','off','FontSize',18)
xlabel('Time [s]', 'Interpreter', 'latex','FontSize',20) 
ylabel('Image features error', 'Interpreter', 'latex','FontSize',20) 
xlim([0.5 test_time])
ylim([-0.3 0.8])