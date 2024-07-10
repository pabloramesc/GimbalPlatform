clear; close all; clc;

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
dt = data(:,14); % BLE33 internal clock dt (ms)

ang = [tx ty tz];
acc = [ax ay -az];
gyr = [gx gy -gz];
mag = [-mx my -mz];

dt = dt/1000.0; % dt in seconds
t = t - t(1); % time vector starting in 0s

acc_var = var(acc,0,1); % compute sensors variance
mag_var = var(mag,0,1);
gyr_var = var(gyr,0,1);
fprintf('acc var = %f %f %f\n',acc_var);
fprintf('mag var = %f %f %f\n',mag_var);
fprintf('gyr var = %f %f %f\n',gyr_var);

acc_sig = sqrt(acc_var); % compute sensors standar deviation
mag_sig = sqrt(mag_var);
gyr_sig = sqrt(gyr_var);
fprintf('acc sig = %f %f %f\n',acc_sig);
fprintf('mag sig = %f %f %f\n',mag_sig);
fprintf('gyr sig = %f %f %f\n',gyr_sig);

acc_lpf = low_pass_filter(acc, 0.6); % low pass filtered accel data


figure()
subplot(4,1,1)
title('Platform data')
ylabel('Angles (deg)')
hold on
plot(ang(:,1))
plot(ang(:,2))
plot(ang(:,3))
hold off
legend('tx','ty','tz')
subplot(4,1,2)
ylabel('Accel (g)')
hold on
plot(acc(:,1))
plot(acc(:,2))
plot(acc(:,3))
hold off
legend('ax','ay','az')
subplot(4,1,3)
ylabel('Gyro (deg/s)')
hold on
plot(gx)
plot(gy)
plot(gz)
hold off
legend('gx','gy','gz')
subplot(4,1,4)
xlabel('Samples')
ylabel('Mag (uT)')
hold on
plot(mx)
plot(my)
plot(mz)
hold off
legend('mx','my','mz')

figure()
title('BLE33 dt (ms)')
plot(dt)

k1 = 500;
k2 = 600;
figure()
sgtitle('raw accel vs filtered')
subplot(3,1,1)
hold on
plot(acc(k1:k2,1))
plot(acc_lpf(k1:k2,1))
hold off
ylabel('acc x (g)')
legend('raw','filtered')
subplot(3,1,2)
hold on
plot(acc(k1:k2,2))
plot(acc_lpf(k1:k2,2))
hold off
ylabel('acc y (g)')
legend('raw','filtered')
subplot(3,1,3)
hold on
plot(acc(k1:k2,3))
plot(acc_lpf(k1:k2,3))
hold off
ylabel('acc z (g)')
xlabel('samples')
legend('raw','filtered')

figure()
title ('magnetometer XY data')
xlabel('mx')
ylabel('my')
hold on
plot(mx, my, '.')
hold off

figure()
title ('magnetometer XZ data')
xlabel('mx')
ylabel('mz')
hold on
plot(mx, mz, '.')
hold off

figure()
title ('magnetometer YZ data')
xlabel('my')
ylabel('mz')
hold on
plot(my, mz, '.')
hold off

figure()
title ('magnetometer 3D data')
hold on
plot3(mx, my, mz, '.')
hold off