clear all, close all, clc

Ts=0.1;
N=75000;
time=[1:N]*Ts;
a=7400; % semi-major axis (km)
mu=3.986*10^5; % gravitational constant
n=sqrt(mu/(a^3)); % angular velocity

% Hill formation dynamic matrix [x,xdot, y, ydot, z , zdot]
A = [0,1,0,0,0,0;
     0,0,0,2*n,0,0;
     0,0,0,1,0,0;
     0,-2*n,3*n^2,0,0,0;
     0,0,0,0,0,1;
     0,0,0,0,-n^2,0];

% Initail states for desired trajectory(Projected circular formation)
% x-z plot will be a circle with 200 km radius
xd = zeros(6,N);
xd(:,1) = [-2*(-69*n)/n;       % x
           2*n*(-69);          % xdot
           -69;                % y
           -69*n;              % ydot
           -2*(-69);           % z
           -2*(-69)*n];        % zdot

% Initial follower state
x = zeros(6,N);
x(:,1) = [0; 0; 0; -0.5; 0; 1]; % [x xdot y ydot z zdot]

% Control input initialization
u = zeros(3,N);

% Controller matrices
B = [0 0 0;
     1 0 0;
     0 0 0;
     0 1 0;
     0 0 0;
     0 0 1];

Q = 1e-8 * diag([1,1,1,1,1,1]);
R = diag([1,1,1]);
[K,~,~] = lqr(A,B,Q,R); % LQR gain matrix

% Simulation loop
for t = 2:N
    % Desired trajectory dynamics
    xd(:,t) = xd(:,t-1) + Ts * A * xd(:,t-1);

    % Follower dynamics
    x(:,t) = x(:,t-1) + Ts * (A * x(:,t-1) + B * u(:,t-1));

    % Tracking error
    er(:,t) = x(:,t) - xd(:,t);

    % Control law
    u(:,t) = -K * er(:,t);

end

Plot_SFF

% % Plotting
% figure (1) % Desired x,y,z motion
% plot(time,xd(1,:),'r','LineWidth',1.5); hold on
% plot(time,xd(3,:),'g','LineWidth',1.5);
% plot(time,xd(5,:),'b','LineWidth',1.5);
% grid on; legend('x_d','y_d','z_d');
% 
% figure (2) % Desired 3D motion
% plot3(xd(1,:),xd(3,:),xd(5,:));
% xlabel('X,(km)'); ylabel('Y,(km)'); zlabel('Z,(km)'); grid on;
% 
% figure (3) % Desired projected circular formation on x-z plane
% plot(xd(1,:),xd(5,:));
% xlabel('Xd,(km)'); ylabel('Zd,(km)'); grid on;
% 
% figure (4) % Follower 3D motion
% plot3(x(1,:),x(3,:),x(5,:));
% xlabel('X,(km)'); ylabel('Y,(km)'); zlabel('Z,(km)'); grid on;
% 
% figure (5) % Follower x,y,z motion
% plot(time,x(1,:),'r','LineWidth',1.5); hold on
% plot(time,x(3,:),'g','LineWidth',1.5);
% plot(time,x(5,:),'b','LineWidth',1.5);
% grid on; legend('x','y','z');
% 
% figure (6) % Error
% plot(time,er(1,:),'r','LineWidth',1.5); hold on
% plot(time,er(3,:),'g','LineWidth',1.5);
% plot(time,er(5,:),'b','LineWidth',1.5);
% grid on; legend('e_x','e_y','e_z');
% 
% figure (7) % Follower projected circular formation on x-z plane
% plot(x(1,:),x(5,:));
% xlabel('X,(km)'); ylabel('Z,(km)'); grid on;
% 
% figure (8) % Control inputs
% plot(time,u(1,:),'r','LineWidth',1.5); hold on
% plot(time,u(2,:),'g','LineWidth',1.5);
% plot(time,u(3,:),'b','LineWidth',1.5);
% grid on; legend('u_x','u_y','u_z');
% 
% figure (9)
% subplot(2,2,1)
% plot(x(1,:),x(3,:)); xlabel('X,km'); ylabel('Y,km');
% subplot(2,2,2)
% plot(x(1,:),x(5,:)); xlabel('X,km'); ylabel('Z,km');
% subplot(2,2,3)
% plot(x(3,:),x(5,:)); xlabel('Y,km'); ylabel('Z,km');
% subplot(2,2,4)
% plot3(x(1,:),x(3,:),x(5,:));
% xlabel('X,(km)'); ylabel('Y,(km)'); zlabel('Z,(km)');
