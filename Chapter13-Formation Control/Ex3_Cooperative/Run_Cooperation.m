clc; clear; close all;

Sim_time = 180;
dt = 0.01;

numRobots = 4;              % Number of robots
radius = 10;                 % Desired radius of the circle
px = 10;
py = 10;
u0 = 1;

%%%%% params
DS = 1; %m distance to stop
MaxF = 45; % N max F
MinF = 15;
SR = 30; % speed ratio, for example 45/30=1.5m/s
tau_s = 1; % speed time constant

M_psi_r = 20*pi/180; % psi change rate

delta_r_max = 30*pi/180; % delta_r_max max delta saturation

% xAUV_ini = [randi([-1 ,2],1,1), randi([-10 -5],1,1), randi([5 ,8],1,1),randi([11 ,14],1,1)];
% yAUV_ini = randi([0 ,15],numRobots,1);

xAUV_ini = [2, 15, 6, 20];
yAUV_ini = [15, -20, -5, 0];

targetX = [100,110,80,95];
targetY = [100, 120,130,125];

Xob = [35,50,80,50];
Yob = [35,50,80,100];
% Robot initial positions in a circle



% % Robot desired location 
% theta = linspace(0, 2*pi, numRobots+1); % divide circle to num
% theta = theta(1:end-1);


% open('Cooperative')
sim('Cooperative')


figure
plot_APF_Coop;
