%% Program to design the Lead compensator for a given system 
% Consider a numerical from Advanced Control Theory,
% Numerical 1.6: Design a lead compensator for a unity feedback system with
% open loop transfer function G(s)= K/(s(s+1)(s+5)) to satisfy the
% following specifications (i) velocity error constant K_v=50, 
 

 
close all
clear all
clc
 
%% Uncompensated control system transfer function (assuming H(s)=1)
% Calculate the value of gain K from e_ss and use it. 
num=[10];
den=[1 10 16 0];
G=tf(num,den);
 
%% Bode plot of the uncompensated system 
figure(1)
bode(G), grid on            % To check PM and GM of the unc5ompensated system 
title('Bode plot of uncompensated system')
[Gm,Pm,Wcg,Wcp] = margin(G);
 
%% Lead compensator Design
Pmd=20;                       % Desired Phase Margin (PM)
Phi_m=Pmd-Pm+30;               % Maximum phase lead angle (degree)
% Check different values for the safety factor to get the desired PM
 
Phi_mr=Phi_m*(pi/180);        % Maximum phase lead angle (radian)
% Determine the transfer function of the lead compensator 
alpha=(1+sin(Phi_mr))/(1-sin(Phi_mr));
Mc=-10*log10(alpha);          % Magnitude to be compensated in db
 
% Locate the frequency in Figure(1) for Mc
wm=28.5;
p=wm*sqrt(alpha);             % Pole of lead compensator
z=p/alpha;                    % Zero of lead compensator
gain=alpha;
numc=[1 z];
denc=[1 p];
Gc=tf(numc,denc);
 
% Total forward transfer function of the compensated system
Gt=gain*Gc*G;
 
%% Comparison of compensated and uncompensated bode plots
figure(2)
bode(G,'--r', Gt,'-'), grid on
legend('Uncompensated system', 'Compensated system')
title('Comparison of compensated and uncompensated bode plots')
 
%% Since H(s)=1, the feedback transfer function 
Gc1u=feedback(G,1);         % Closed loop TF of uncompensated system
Gclc=feedback(Gt,1);        % Closed loop TF of compensated system
 
% Comparison of compensated and uncompensated step responses
figure(3)
subplot(2,1,1)
step(Gc1u, '--r'); grid on  
title('Uncompensated system step response')

subplot(2,1,2)
step(Gclc, '-'); grid on  
title('Compensated system step response')
%% Comparison of compensated and uncompensated ramp responses
t=0:0.002:2;
figure(4)
[y1,z1]=step(250, [1 6 5 250 0],t);   % Take num and den coefficients of Gclu with a pole at origin
[y2,z2]=step([2.185e06 6.661e05], [1 2670 1.599e04 2.198e06 6.661e05 0],t);
                                % Take num and den coefficients of Gclc with a pole at the origin
subplot(2,1,1)
plot(t,y1,'.'), grid on
title('Uncompensated system ramp response')

subplot(2,1,2)
plot(t,y2,'-'), grid on
title('Compensated system ramp response')