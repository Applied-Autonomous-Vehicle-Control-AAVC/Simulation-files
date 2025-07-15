clear all, close all, clc

Sim_Time = 200;
dt = 0.001;


Xd = [150; 90];           % Goal
Xob = [75 80; 40 65];     % Two obstacles (columns)

u_min = 0.2;
u_max = 2.5;

u0 = 1;
psi_r_sat = 10*pi/180; %10
delta_max = 500*pi/180; %30

%open('Formation_2_obs.slx')
sim('Formation_2_obs.slx')

Plotting_figures