% Plot outputs
close all

f_size_lab = 14;
f_size_ticks = 12;
f_size_legend = 12;

L=0.1;  R=5;
% Xd=[100 ;60 ];     % goal position
Xob1 = Xob(:,1);  % obstacle 1
Xob2 = Xob(:,2);  % obstacle 2    

%%  Plot Obstacles and Goal
n=0:0.1:2*pi;  
% obstacle 1
a1=Xob1(1);
b1=Xob1(2);
% r=R;
% xx=(r-2).*sin(n)+a;
% yy=(r-2).*cos(n)+b;
% xb=R.*sin(n)+a;
% yb=R.*cos(n)+b;
% obstacle 2
a2=Xob2(1);
b2=Xob2(2);

%% for correct legend 
figure(1)
plot(a1,b1,'o','MarkerSize',25,'markerfacecolor','k');
hold on
plot(a2,b2,'o','MarkerSize',25,'markerfacecolor','k');
plot(Xd(1), Xd(2), 'p', 'MarkerSize', 30, 'Color', [0.6 0.6 0.6], 'LineWidth', 1.5)
%% Plot AUVs trajectories
LL = length(Leader);  % length vector
figure(1)
hold on
plot(Leader(:,1),Leader(:,2),'--b', 'LineWidth',2)
plot(PF1_bar(:,1),PF1_bar(:,2),'--r', 'LineWidth',2)
plot(PF2_bar(:,1),PF2_bar(:,2),'--g', 'LineWidth',2)

plot(Leader(LL,1),Leader(LL,2),'p','MarkerSize',9,'MarkerEdgeColor','b')
plot(PF1_bar(LL,1),PF1_bar(LL,2),'*','MarkerSize',9,'MarkerEdgeColor','r')
plot(PF2_bar(LL,1),PF2_bar(LL,2),'o','MarkerSize',9,'MarkerEdgeColor','g')

xlabel('$x (m)$','FontSize',f_size_lab,'Interpreter','latex')
ylabel('$y (m)$','FontSize',f_size_lab,'Interpreter','latex')

%%  plot AUVS
plot(Leader(1,1),Leader(1,2),'p','MarkerSize',9,'MarkerEdgeColor','b')
plot(PF1_bar(1,1),PF1_bar(1,2),'*','MarkerSize',9,'MarkerEdgeColor','r')
plot(PF2_bar(1,1),PF2_bar(1,2),'o','MarkerSize',9,'MarkerEdgeColor','g')
       X = [Leader(1,1),PF1_bar(1,1)];
       Y = [Leader(1,2),PF1_bar(1,2)];
       plot(X,Y,'y','LineWidth' ,1)
       hold on 
       X = [Leader(1,1),PF2_bar(1,1)];
       Y = [Leader(1,2),PF2_bar(1,2)];
       plot(X,Y,'m','LineWidth' ,1)
       X = [PF1_bar(1,1),PF2_bar(1,1)];
       Y = [PF1_bar(1,2),PF2_bar(1,2)];
       plot(X,Y,'m','LineWidth' ,1)
       hold on


a = fix(0.15*LL);

plot(Leader(a,1),Leader(a,2),'p','MarkerSize',9,'MarkerEdgeColor','b')
plot(PF1_bar(a,1),PF1_bar(a,2),'*','MarkerSize',9,'MarkerEdgeColor','r')
plot(PF2_bar(a,1),PF2_bar(a,2),'o','MarkerSize',9,'MarkerEdgeColor','g')
       X = [Leader(a,1),PF1_bar(a,1)];
       Y = [Leader(a,2),PF1_bar(a,2)];
       plot(X,Y,'y','LineWidth' ,1)
       hold on 
       X = [Leader(a,1),PF2_bar(a,1)];
       Y = [Leader(a,2),PF2_bar(a,2)];
       plot(X,Y,'m','LineWidth' ,1)
       X = [PF1_bar(a,1),PF2_bar(a,1)];
       Y = [PF1_bar(a,2),PF2_bar(a,2)];
       plot(X,Y,'m','LineWidth' ,1)
       hold on

a = fix(0.25*LL);
plot(Leader(a,1),Leader(a,2),'p','MarkerSize',9,'MarkerEdgeColor','b')
plot(PF1_bar(a,1),PF1_bar(a,2),'*','MarkerSize',9,'MarkerEdgeColor','r')
plot(PF2_bar(a,1),PF2_bar(a,2),'o','MarkerSize',9,'MarkerEdgeColor','g')
       X = [Leader(a,1),PF1_bar(a,1)];
       Y = [Leader(a,2),PF1_bar(a,2)];
       plot(X,Y,'y','LineWidth' ,1)
       hold on 
       X = [Leader(a,1),PF2_bar(a,1)];
       Y = [Leader(a,2),PF2_bar(a,2)];
       plot(X,Y,'m','LineWidth' ,1)
       X = [PF1_bar(a,1),PF2_bar(a,1)];
       Y = [PF1_bar(a,2),PF2_bar(a,2)];
       plot(X,Y,'m','LineWidth' ,1)
       hold on
 
a = fix(0.35*LL);
plot(Leader(a,1),Leader(a,2),'p','MarkerSize',9,'MarkerEdgeColor','b')
plot(PF1_bar(a,1),PF1_bar(a,2),'*','MarkerSize',9,'MarkerEdgeColor','r')
plot(PF2_bar(a,1),PF2_bar(a,2),'o','MarkerSize',9,'MarkerEdgeColor','g')
       X = [Leader(a,1),PF1_bar(a,1)];
       Y = [Leader(a,2),PF1_bar(a,2)];
       plot(X,Y,'y','LineWidth' ,1)
       hold on 
       X = [Leader(a,1),PF2_bar(a,1)];
       Y = [Leader(a,2),PF2_bar(a,2)];
       plot(X,Y,'m','LineWidth' ,1)
       X = [PF1_bar(a,1),PF2_bar(a,1)];
       Y = [PF1_bar(a,2),PF2_bar(a,2)];
       plot(X,Y,'m','LineWidth' ,1)
       hold on

a = fix(0.35*LL);
plot(Leader(a,1),Leader(a,2),'p','MarkerSize',9,'MarkerEdgeColor','b')
plot(PF1_bar(a,1),PF1_bar(a,2),'*','MarkerSize',9,'MarkerEdgeColor','r')
plot(PF2_bar(a,1),PF2_bar(a,2),'o','MarkerSize',9,'MarkerEdgeColor','g')
       X = [Leader(a,1),PF1_bar(a,1)];
       Y = [Leader(a,2),PF1_bar(a,2)];
       plot(X,Y,'y','LineWidth' ,1)
       hold on 
       X = [Leader(a,1),PF2_bar(a,1)];
       Y = [Leader(a,2),PF2_bar(a,2)];
       plot(X,Y,'m','LineWidth' ,1)
       X = [PF1_bar(a,1),PF2_bar(a,1)];
       Y = [PF1_bar(a,2),PF2_bar(a,2)];
       plot(X,Y,'m','LineWidth' ,1)
       hold on       

a = fix(0.45*LL);
plot(Leader(a,1),Leader(a,2),'p','MarkerSize',9,'MarkerEdgeColor','b')
plot(PF1_bar(a,1),PF1_bar(a,2),'*','MarkerSize',9,'MarkerEdgeColor','r')
plot(PF2_bar(a,1),PF2_bar(a,2),'o','MarkerSize',9,'MarkerEdgeColor','g')
       X = [Leader(a,1),PF1_bar(a,1)];
       Y = [Leader(a,2),PF1_bar(a,2)];
       plot(X,Y,'y','LineWidth' ,1)
       hold on 
       X = [Leader(a,1),PF2_bar(a,1)];
       Y = [Leader(a,2),PF2_bar(a,2)];
       plot(X,Y,'m','LineWidth' ,1)
       X = [PF1_bar(a,1),PF2_bar(a,1)];
       Y = [PF1_bar(a,2),PF2_bar(a,2)];
       plot(X,Y,'m','LineWidth' ,1)
       hold on

a = fix(0.55*LL);
plot(Leader(a,1),Leader(a,2),'p','MarkerSize',9,'MarkerEdgeColor','b')
plot(PF1_bar(a,1),PF1_bar(a,2),'*','MarkerSize',9,'MarkerEdgeColor','r')
plot(PF2_bar(a,1),PF2_bar(a,2),'o','MarkerSize',9,'MarkerEdgeColor','g')
       X = [Leader(a,1),PF1_bar(a,1)];
       Y = [Leader(a,2),PF1_bar(a,2)];
       plot(X,Y,'y','LineWidth' ,1)
       hold on 
       X = [Leader(a,1),PF2_bar(a,1)];
       Y = [Leader(a,2),PF2_bar(a,2)];
       plot(X,Y,'m','LineWidth' ,1)
       X = [PF1_bar(a,1),PF2_bar(a,1)];
       Y = [PF1_bar(a,2),PF2_bar(a,2)];
       plot(X,Y,'m','LineWidth' ,1)
       hold on
       
a = fix(0.7*LL);
plot(Leader(a,1),Leader(a,2),'p','MarkerSize',9,'MarkerEdgeColor','b')
plot(PF1_bar(a,1),PF1_bar(a,2),'*','MarkerSize',9,'MarkerEdgeColor','r')
plot(PF2_bar(a,1),PF2_bar(a,2),'o','MarkerSize',9,'MarkerEdgeColor','g')
       X = [Leader(a,1),PF1_bar(a,1)];
       Y = [Leader(a,2),PF1_bar(a,2)];
       plot(X,Y,'y','LineWidth' ,1)
       hold on 
       X = [Leader(a,1),PF2_bar(a,1)];
       Y = [Leader(a,2),PF2_bar(a,2)];
       plot(X,Y,'m','LineWidth' ,1)
       X = [PF1_bar(a,1),PF2_bar(a,1)];
       Y = [PF1_bar(a,2),PF2_bar(a,2)];
       plot(X,Y,'m','LineWidth' ,1)
       hold on
       
a = fix(0.8*LL);
plot(Leader(a,1),Leader(a,2),'p','MarkerSize',9,'MarkerEdgeColor','b')
plot(PF1_bar(a,1),PF1_bar(a,2),'*','MarkerSize',9,'MarkerEdgeColor','r')
plot(PF2_bar(a,1),PF2_bar(a,2),'o','MarkerSize',9,'MarkerEdgeColor','g')
       X = [Leader(a,1),PF1_bar(a,1)];
       Y = [Leader(a,2),PF1_bar(a,2)];
       plot(X,Y,'y','LineWidth' ,1)
       hold on 
       X = [Leader(a,1),PF2_bar(a,1)];
       Y = [Leader(a,2),PF2_bar(a,2)];
       plot(X,Y,'m','LineWidth' ,1)
       X = [PF1_bar(a,1),PF2_bar(a,1)];
       Y = [PF1_bar(a,2),PF2_bar(a,2)];
       plot(X,Y,'m','LineWidth' ,1)
       hold on
       
a = fix(0.9*LL);
plot(Leader(a,1),Leader(a,2),'p','MarkerSize',9,'MarkerEdgeColor','b')
plot(PF1_bar(a,1),PF1_bar(a,2),'*','MarkerSize',9,'MarkerEdgeColor','r')
plot(PF2_bar(a,1),PF2_bar(a,2),'o','MarkerSize',9,'MarkerEdgeColor','g')

       X = [Leader(a,1),PF1_bar(a,1)];
       Y = [Leader(a,2),PF1_bar(a,2)];
       plot(X,Y,'y','LineWidth' ,1)
       hold on 
       X = [Leader(a,1),PF2_bar(a,1)];
       Y = [Leader(a,2),PF2_bar(a,2)];
       plot(X,Y,'m','LineWidth' ,1)
       X = [PF1_bar(a,1),PF2_bar(a,1)];
       Y = [PF1_bar(a,2),PF2_bar(a,2)];
       plot(X,Y,'m','LineWidth' ,1)
       hold on
       X = [Leader(LL,1),PF1_bar(LL,1)];
       Y = [Leader(LL,2),PF1_bar(LL,2)];
       plot(X,Y,'y','LineWidth' ,1)
       hold on 
       X = [Leader(LL,1),PF2_bar(LL,1)];
       Y = [Leader(LL,2),PF2_bar(LL,2)];
       plot(X,Y,'m','LineWidth' ,1)
       X = [PF1_bar(LL,1),PF2_bar(LL,1)];
       Y = [PF1_bar(LL,2),PF2_bar(LL,2)];
       plot(X,Y,'m','LineWidth' ,1)
       hold on       
     legend('off')

% a=Xob(1);
% b=Xob(2);     
p1 = plot(Leader(a,1),Leader(a,2),'p','MarkerSize',9,'MarkerEdgeColor','b');
p2 = plot(PF1_bar(a,1),PF1_bar(a,2),'*','MarkerSize',9,'MarkerEdgeColor','r');
p3 = plot(PF2_bar(a,1),PF2_bar(a,2),'o','MarkerSize',9,'MarkerEdgeColor','g');
legend([p1 p2 p3],{'Leader','Follower1','Follower2'}, ...
       'FontSize', f_size_legend, 'Interpreter', 'latex', 'Location','best')
set(gca, 'FontSize', f_size_ticks, 'TickLabelInterpreter', 'latex')
text(148, 99, 'Goal', 'FontSize', f_size_lab, 'Interpreter', 'latex', ...
     'Color', [0.5 0.5 0.5], 'FontWeight', 'bold')

xlim([-5,165])
ylim([-5,105])

fig_pos = get(gcf, 'Position');         % Get current figure position
fig_pos(1) = fig_pos(1) - 100;          % Shift left by 100 pixels
fig_pos(2) = fig_pos(2) - 100;          % Shift down by 100 pixels
fig_pos(3:4) = fig_pos(3:4) * 1.2;      % Increase width and height by 1.5×
set(gcf, 'Position', fig_pos);         % Apply new position and size


%%
%%%%%%%%%% Second Figure %%%%%%%%%%%

figure(2)

% Subplot 1
subplot(2,2,1)
plot(tout ,L12(:,1),'r:', 'LineWidth',1.5)
hold on
plot(tout ,L12(:,2),'k', 'LineWidth',1.5)
axis([0 tout(end) 0 inf])
ylabel('$\ell_{12}$ (m)','FontSize',f_size_lab,'Interpreter','latex')
legend({'$\ell_{12}^{\mathrm{d}}$','$\ell_{12}$'}, ...
       'FontSize', f_size_legend, 'Interpreter', 'latex', 'Location','best')
set(gca, 'FontSize', f_size_ticks, 'TickLabelInterpreter', 'latex')
title('$\ell$--$\psi$ Controller', 'Interpreter', 'latex', 'FontSize', 10)
ylim([0,8])

% Subplot 2
subplot(2,2,2)
plot(tout ,psi12(:,1),'r:', 'LineWidth',1.5)
hold on
plot(tout ,psi12(:,2),'k', 'LineWidth',1.5)
axis([0 tout(end) 0 inf])
ylabel('$\psi_{12}$ (rad)','FontSize',f_size_lab,'Interpreter','latex')
legend({'$\psi_{12}^{\mathrm{d}}$','$\psi_{12}$'}, ...
       'FontSize', f_size_legend, 'Interpreter', 'latex', 'Location','best')
set(gca, 'FontSize', f_size_ticks, 'TickLabelInterpreter', 'latex')
title('$\ell$--$\psi$ Controller', 'Interpreter', 'latex', 'FontSize', 10)
ylim([0,6])

% Subplot 3
subplot(2,2,3)
plot(tout ,L13(:,1),'r:', 'LineWidth',1.5)
hold on
plot(tout ,L13(:,2),'k', 'LineWidth',1.5)
axis([0 tout(end) 0 inf])
xlabel('Time (s)','FontSize',f_size_lab,'Interpreter','latex')
ylabel('$\ell_{13}$ (m)','FontSize',f_size_lab,'Interpreter','latex')
legend({'$\ell_{13}^{\mathrm{d}}$','$\ell_{13}$'}, ...
       'FontSize', f_size_legend, 'Interpreter', 'latex', 'Location','best')
set(gca, 'FontSize', f_size_ticks, 'TickLabelInterpreter', 'latex')
title('$\ell$--$\ell$ Controller', 'Interpreter', 'latex', 'FontSize', 10)
ylim([0,8])

% Subplot 4
subplot(2,2,4)
plot(tout ,L23(:,1),'r:', 'LineWidth',1.5)
hold on
plot(tout ,L23(:,2),'k', 'LineWidth',1.5)
axis([0 tout(end) 0 inf])
xlabel('Time (s)','FontSize',f_size_lab,'Interpreter','latex')
ylabel('$\ell_{23}$ (m)','FontSize',f_size_lab,'Interpreter','latex')
legend({'$\ell_{23}^{\mathrm{d}}$','$\ell_{23}$'}, ...
       'FontSize', f_size_legend, 'Interpreter', 'latex', 'Location','best')
set(gca, 'FontSize', f_size_ticks, 'TickLabelInterpreter', 'latex')
title('$\ell$--$\ell$ Controller', 'Interpreter', 'latex', 'FontSize', 10)
ylim([0,10])
