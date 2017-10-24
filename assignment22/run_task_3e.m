clear all

deg2rad = pi/180;   
rad2deg = 180/pi;

%% define system
A = [[-0.322, 0.052, 0.028, -1.12, 0.002], 
 [0, 0, 1, -0.001, 0],
 [-10.6, 0, -2.87, 0.46, -0.65],
 [6.87, 0, -0.04, -0.32, -0.02],
 [0, 0, 0, 0, -10]];

B = [0, 0, 0, 0, 10]'; % from hint: "so the B-matrix in the Kalman filter will be nonzero for every entry except one."

C = [[0, 0, 0, 1, 0],
  [0, 0, 1, 0 ,0],
  [1, 0, 0, 0, 0],
  [0 ,1 ,0, 0, 0]];

C2 = [[0, 0, 0, 1, 0],
  [0, 0, 1, 0 ,0],
  [0, 0, 0, 0, 0],
  [0 ,1 ,0, 0, 0]];

C3 = [[0, 0, 0, 1, 0], % no reason to include a row of zeros as in C2. now dim(y)= 3x1
  [0, 0, 1, 0 ,0],
  [0 ,1 ,0, 0, 0]];

D = [0, 0, 0, 0, 0]';

D3 = [0,0,0]';
%D = zeros(4,5);

Tl = 0.1;
kd = 2.153;
kp = -1.67;
ki = 0;

g  = 9.81;
Vg = 637/3.6;
zeta_chi = 0.8;
omega_chi = 0.104;
kpx = 2*zeta_chi*omega_chi * Vg / g; 
kix = omega_chi^2*Vg / g;

sys_c = ss(A,B,C3,D3);

% Kalman sys
samplingfrequency = 10;
sys_d = c2d(sys_c, 1/samplingfrequency);

Q = 1e-6 * eye(5);

R = 1 * eye(3);

H = sys_d.C;

xhat_0 = [0,0,0,0,0]';

P_0 = eye(5) * 5 * deg2rad;

A_d = sys_d.A;
B_d = sys_d.B;


save('vars','A_d','Q', 'B_d', 'H', 'R');

%% Noise
r_noise = 0.2^2 * deg2rad^2;
p_noise = 0.5^2 * deg2rad^2;
phi_noise = 2^2 * deg2rad^2;

disturbance = 10 * deg2rad;


%% simulation 
T = 300;
h = 0.1;
N = T/h +1;
[u1,t] = gensig('square',100,T,h); 
u1 = u1*15*deg2rad; u1(((N-1)/2):N) = u1(((N-1)/2):N)+10*deg2rad;

scale = 0.1;
lowpass = tf((1*scale/(2*pi*omega_chi))^2,[1 2*(1*scale/(2*pi*omega_chi)), (1*scale/(2*pi*omega_chi))^2]);

disturbance = 0;%10*deg2rad;

u2 = lsim(lowpass,u1,t);
simin = [t, u2];
sim('sys_with_kalman_2016.slx')


%% plot simulations
close all

figure()
%COURSE
plot(chi*rad2deg)
hold on
plot(chi_ref*rad2deg)
legend('course','reference')
grid on
title('Course reference and signal')
ylabel('Chi [deg]')
xlabel('time')

figure()
%Aileron
plot(x.time, x.data(:,5)*rad2deg)
grid on
title('Aileron input delta')
ylabel('deg')
xlabel('time')

figure()
plot(y_measured.time, y_measured.data(:,1)*rad2deg)
hold on
grid on
plot(x.time, x.data(:,4)*rad2deg)
plot(xhat.time, xhat.data(:,4)*rad2deg)
legend('Measured value','True value','Estimated value')
title('r')
ylabel('deg')
xlabel('time')

figure()
plot(y_measured.time, y_measured.data(:,2)*rad2deg)
hold on
grid on
plot(x.time, x.data(:,3)*rad2deg)
plot(xhat.time, xhat.data(:,3)*rad2deg)
legend('Measured value','True value','Estimated value')
title('p')
ylabel('deg')
xlabel('time')

figure()
plot(y_measured.time, y_measured.data(:,3)*rad2deg)
hold on
grid on
plot(x.time, x.data(:,2)*rad2deg)
plot(xhat.time, xhat.data(:,2)*rad2deg)
legend('Measured value','True value','Estimated value')
title('phi')
ylabel('deg')
xlabel('time')



