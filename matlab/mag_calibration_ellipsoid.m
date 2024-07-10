clear; clc; close all;

load('datasets/platform/test.mat');

% measured magnetic field
mx = data(:,8);
my = data(:,9);
mz = data(:,10);
m = [mx; my; mz];

% Magnetometer error model --> m = C*h + b ; A = inv(C)
% Ellipsoidal fitting calibration --> h = A*(m - b)
% h : true local magnetic field
% m : measured magnetic field
% C : soft iron error matrix (transformation)
% b : hard iron error vector (bias)

D = [mx, my, mz];
[A,b,expmfs] = magcal(D)

hx = A(1,1)*(mx - b(1)) + A(1,2)*(my - b(2)) + A(1,3)*(mz - b(3));
hy = A(2,1)*(mx - b(1)) + A(2,2)*(my - b(2)) + A(2,3)*(mz - b(3));
hz = A(3,1)*(mx - b(1)) + A(3,2)*(my - b(2)) + A(3,3)*(mz - b(3));

figure(1)
title ('magnetometer XY data')
xlabel('mx')
ylabel('my')
hold on
plot(mx, my, '.')
plot(hx, hy, '.')
legend('measured','calibrated')
grid on
axis equal
hold off

figure(2)
title ('magnetometer XZ data')
xlabel('mx')
ylabel('mz')
hold on
plot(mx, mz, '.')
plot(hx, hz, '.')
legend('measured','calibrated')
grid on
axis equal
hold off

figure(3)
title ('magnetometer YZ data')
xlabel('my')
ylabel('mz')
hold on
plot(my, mz, '.')
plot(hy, hz, '.')
legend('measured','calibrated')
grid on
axis equal
hold off

figure(4)
title ('magnetometer 3D data')
hold on
plot3(mx, my, mz, '.')
plot3(hx, hy, hz, '.')
legend('measured','calibrated')
grid on
axis equal
hold off