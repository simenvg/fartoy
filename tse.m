a1 = 2.87;
a2 = -0.65;
kd = 2.153;
kp = -1.67;
ki = 0;


%[0:0.1:0.5];
zeta_chi = 0.606;
omega_chi = 1.04;
g  = 9.81;
Vg = 637/3.6;

kpx = 2*zeta_chi*omega_chi * Vg / g; %2.273;
kix = omega_chi^2*Vg / g; %   0.195;

sys_k = tf([kpx, kix], [1,0]);
sys_g = tf([g],[Vg, 0]);
sys_phi = tf([a2*kp, ki/kp],[1, (a1+a2*kd), (a2*kp), a2*ki]);
sys_chi = tf([2*zeta_chi*omega_chi, omega_chi],[1, 2*zeta_chi*omega_chi, omega_chi*omega_chi]);
sys_tot = sys_phi * sys_g * sys_k
sys_tot_cl = feedback(sys_tot, 1);

figure()
margin(sys_tot_cl)
figure()
margin(sys_phi)
figure()
margin(sys_chi)
rlocus(sys)


[u,t] = gensig('sin',1.3); 

lsim(sys_tot_cl,u,t)


