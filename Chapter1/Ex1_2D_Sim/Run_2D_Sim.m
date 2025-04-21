clear; clc; close all;

% Simulation parameters
Sim_time = 92; dt = 0.01; time = 0:dt:Sim_time;
N = length(time);

% Initial conditions
x = zeros(1,N); y = zeros(1,N); psi = zeros(1,N);
V_T = 5; r = zeros(1,N);

r(time >= 20) = 5 * pi / 180;


% Integrate motion equations
for i = 2:N
    % if (i*dt>10 & i*dt<20)
    %     r(i) = 4.5*pi/180;
    % end
    % 
    % if (i*dt>30 & i*dt<40)
    %     r(i) = 4.5*pi/180;
    % end
       
    psi(i) = psi(i-1) + r(i) * dt;
    x(i) = x(i-1) + V_T * cos(psi(i)) * dt;
    y(i) = y(i-1) + V_T * sin(psi(i)) * dt;
end

% Plot results
f_size_lab = 16;
f_size_ticks = 12;

figure;
subplot(2,2,1); 
plot(time, x, 'k', 'LineWidth', 1.5);
ylabel('$x$ (m)', 'Interpreter', 'latex', 'FontSize', f_size_lab); 
xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', f_size_lab);
set(gca, 'FontSize', f_size_ticks, 'TickLabelInterpreter', 'latex');

subplot(2,2,2); 
plot(time, y, 'k', 'LineWidth', 1.5);
ylabel('$\psi$ ($^\circ$)', 'Interpreter', 'latex', 'FontSize', f_size_lab); 
xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', f_size_lab);
set(gca, 'FontSize', f_size_ticks, 'TickLabelInterpreter', 'latex');

subplot(2,2,3); 
plot(time, psi * 180/pi, 'k', 'LineWidth', 1.5);
ylabel('$y$ (m)', 'Interpreter', 'latex', 'FontSize', f_size_lab); 
xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', f_size_lab);
set(gca, 'FontSize', f_size_ticks, 'TickLabelInterpreter', 'latex');

subplot(2,2,4); 
plot(x, y, 'k', 'LineWidth', 1.5);
xlabel('$x$ (m)', 'Interpreter', 'latex', 'FontSize', f_size_lab); 
ylabel('$y$ (m)', 'Interpreter', 'latex', 'FontSize', f_size_lab); 
axis equal;

% Set the axis labels and ticks with LaTeX interpreter
set(gca, 'FontSize', f_size_ticks, 'TickLabelInterpreter', 'latex');
