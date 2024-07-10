clear; clc; close all;

load('datasets/platform/test.mat');

t = data(:,1); % time elapsed (seconds)
tx = data(:,2); % AS5600 measured angles (degrees)
ty = data(:,3);
tz = data(:,4);
ax = data(:,5); % BLE33 accelerometer readings (g)
ay = data(:,6);
az = data(:,7);
gx = data(:,8); % BLE33 gyroscope readings (deg/s)
gy = data(:,9);
gz = data(:,10);
mx = data(:,11); % BLE33 magnetometer readings (uT)
my = data(:,12);
mz = data(:,13);

% change data axis to NED frame
ang = [tx ty -tz];
acc = [ax ay -az]*(9.81); % g to m/s^2
gyr = [gx gy -gz]*(pi/180.0); % deg/S to rad/s
mag = [-mx my -mz]*(100); % uT to gauss

acc = low_pass_filter(acc, 0.6); % filter accel data
mag = low_pass_filter(mag, 0.6); % filter mag data

t = t - t(1); % time vector starting in 0s
dt = gradient(t); % compute dt from t vector

% calibrate magnetometer
mag_bias = mag_cal(mag);
mag = mag - mag_bias; % remove magnetometer offset error

% get initial quaternion to start integration
ang0 = [0 0 0];
q0 = euler2quat(ang0);


%% ATTITUDE FROM ACCEL AND MAGNET

est = accel_magnet_attitude(acc, mag);
est = rad2deg(est);
estims(1,:,:) = est; %store estimation

txt = "Attitude from Accelerometer and Magnetometer";
plotter(t, ang, est, txt);

rms_r = RMS(t, ang(:,1), est(:,1));
rms_p = RMS(t, ang(:,2), est(:,2));
rms_y = RMS(t, ang(:,3), est(:,3));

fprintf("%s \n", txt);
fprintf("RMS_roll = %.4f , RMS_pitch = %.4f , RMS_yaw = %.4f \n\n", rms_r, rms_p, rms_y)

%% GYRO INTEGRATION

quat = gyro_integration(gyr, q0, dt);
est = quat2euler(quat);
est = rad2deg(est);
estims(2,:,:) = est; %store estimation

txt = "Gyroscope Integration";
plotter(t, ang, est, txt);

rms_r = RMS(t, ang(:,1), est(:,1));
rms_p = RMS(t, ang(:,2), est(:,2));
rms_y = RMS(t, ang(:,3), est(:,3));

fprintf("%s \n", txt);
fprintf("RMS_roll = %.4f , RMS_pitch = %.4f , RMS_yaw = %.4f \n\n", rms_r, rms_p, rms_y)

%% COMPLEMENTARY FILTER

gain = 0.5;
quat = complementary_filter(acc, mag, gyr, q0, dt, gain);
est = quat2euler(quat);
est = rad2deg(est);
estims(3,:,:) = est; %store estimation

txt = "Complementary Filter";
plotter(t, ang, est, txt);

rms_r = RMS(t, ang(:,1), est(:,1));
rms_p = RMS(t, ang(:,2), est(:,2));
rms_y = RMS(t, ang(:,3), est(:,3));

fprintf("%s \n", txt);
fprintf("RMS_roll = %.4f , RMS_pitch = %.4f , RMS_yaw = %.4f \n\n", rms_r, rms_p, rms_y)

%% Extended Kalman Filter

noises = [0.3 20 0.8]; % standar deviation
quat = extended_kalman_filter(acc, mag, gyr, q0, dt, noises);
est = quat2euler(quat);
est = rad2deg(est);
estims(4,:,:) = est; %store estimation

txt = "Extended Kalman Filter";
plotter(t, ang, est, txt);

rms_r = RMS(t, ang(:,1), est(:,1));
rms_p = RMS(t, ang(:,2), est(:,2));
rms_y = RMS(t, ang(:,3), est(:,3));

fprintf("%s \n", txt);
fprintf("RMS_roll = %.4f , RMS_pitch = %.4f , RMS_yaw = %.4f \n\n", rms_r, rms_p, rms_y)

%% Compare Estimators
k1 = 500;
k2 = 1000;
comparer(t(k1:k2), ang(k1:k2,:), estims(:,k1:k2,:), 'Filters Error Comparison', {'Acc+Mag', 'Gyro', 'Comp', 'EKF'})

%% Auxiliary Functions
function plotter(t, true, estim, title)
    figure()
    sgtitle(sprintf(title))
    subplot(3,1,1)
    hold on
    plot(t, estim(:,1))
    plot(t, true(:,1))
    hold off
    legend('estimated','true')
    grid on; set(gca,'ytick',[-180:45:180])
    ylabel('Roll: \phi (deg)')
    subplot(3,1,2)
    hold on
    plot(t, estim(:,2))
    plot(t, true(:,2))
    hold off
    legend('estimated','true')
    grid on; set(gca,'ytick',[-90:45:90])
    ylabel('Pitch: \theta (deg)')
    subplot(3,1,3)
    hold on
    plot(t, estim(:,3))
    plot(t, true(:,3))
    hold off
    legend('estimated','true')
    grid on; set(gca,'ytick',[-360:90:360])
    ylabel('Yaw: \psi (deg)')
    xlabel('time (s)')
end

function comparer(t, true, estims, title, legendtxt)
    N = size(estims, 1);
    figure()
    sgtitle(title)
    subplot(3,1,1)
    ylabel('Roll Error: \Delta\phi (deg)')
    hold on; grid on
    subplot(3,1,2)
    ylabel('Pitch Error: \Delta\theta (deg)')
    hold on; grid on
    subplot(3,1,3)
    ylabel('Yaw Error: \Delta\psi (deg)')
    hold on; grid on
    for k = 1:N
        estim = squeeze(estims(k,:,:));
        subplot(3,1,1)
        plot(t, estim(:,1) - true(:,1), 'LineWidth', 1)
        subplot(3,1,2)
        plot(t, estim(:,2) - true(:,2), 'LineWidth', 1)
        subplot(3,1,3)
        plot(t, estim(:,3) - true(:,3), 'LineWidth', 1)
    end
    subplot(3,1,1)
    legend(legendtxt)
    subplot(3,1,2)
    legend(legendtxt)
    subplot(3,1,3)
    legend(legendtxt)
    xlabel('time (s)')
end

function rms = RMS(t, true, estim)
    N = size(t, 1);
    err = 0;
    for k = 1:N
        err = err + (estim(k) - true(k))^2;
    end
    rms = sqrt(err/N);
end
