% Modeling and Simulation of Aerospace Systems
% Academic Year 2023/2024
% Assignment #2
% Mutti Marcello 220252 10698636

%% EX1
clearvars; close all; clc;

% Set the default font size for axes labels and tick labels
set(0,'DefaultAxesFontSize',25);

% Set the default font size for legend
set(0,'DefaultLegendFontSize',25);

% Set the default linewidth
set(0,'DefaultLineLineWidth',2);

% Set the default text and legend interpreter
set(groot,'defaultTextInterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

% Steady state boundary conditions
T0=20+273.15;   % external wall temperature
Tf=1000+273.15; % internal wall temperature

% Event time instants
ti=[0 1 60];

% Integration time interval
t_int=[ti(1) ti(end)];

% Layers thicknesses [m]
l=1e-3*[5 20 0.1 7 5];

% Divergent nozzle section sizing [m]
D_t=0.0172; % throat diameter
D_e=0.3217; % exit diameter
L=0.5681;   % divergent length
    
% Internal nozzle surface area [m^2]
S=pi*L*(D_e+D_t)/2+pi*((D_t/2)^2+(D_e/2)^2);

% Layers conductivities [W/m*K]
k=[115 284 55e-3 1.75 15.2];

% Densities [kg/m^3]
rho2=8810;  % conductor
rho4=6047;  % insulator

% Specific heat [J/kg*K] 
cp2=381.8;  % conductor
cp4=495;    % insulator

% Masses [kg]
m2=S*l(2)*rho2; % conductor
m4=S*l(4)*rho4; % insulator

% Thermal capacities [J/K]
c2=m2*cp2;  % conductor
c4=m4*cp4;  % insulator

% Thermal Resistances [K/W]
R=l./(S*k);

% Linear Sistem RHS
A=[-(1/(R(1)+R(2)/2)+1/(R(2)/2+R(3)+R(4)/2))/c2 (1/(R(2)/2+R(3)+R(4)/2))/c2;
   (1/(R(2)/2+R(3)+R(4)/2))/c4 -(1/(R(2)/2+R(3)+R(4)/2)+1/(R(4)/2+R(5)))/c4];

B=[1/((R(1)+R(2)/2)*c2) 0;
   0 1/((R(4)/2+R(5))*c4)];

fun=@(t,x) A*x+B*[Ti(T0,Tf,ti,t); T0];

% Eigenvalues computation and stiffness check
lambda=eig(A);
fprintf('System eigenvalues: \n %.5f \n %.5f \n Eigenvalues ratio: %.2f \n',lambda(1),lambda(2),lambda(1)/lambda(2))

% Modified options for ODE integrators
AbsTol=1e-6;    % absolute tolerance
RelTol=1e-8;    % relative tolerance
options=odeset('RelTol',RelTol,'AbsTol',AbsTol);

% Simscape acausal simulation
SIM=sim("rocket_nozzle_acausal_thermal_simulation.slx");
tt=SIM.tout;    % time grid
TS1=SIM.TS1;    % 1N temperature profiles
TS2=SIM.TS2;    % 2N temperature profiles

% Matlab causal ODE solution
[~,TT]=ode45(@(t,x) fun(t,x),tt,T0*ones(2,1),options);

% Causal Algebraic Equations
TN=zeros(length(TT),5);
TN(:,2)=TT(:,1);    % node n2
TN(:,4)=TT(:,2);    % node n4
TN(:,1)=R(1)/2*((2/R(1)-1/(R(1)+R(2)/2)).*Ti(T0,Tf,ti,tt)+(1/(R(1)+R(2)/2)).*TN(:,2));                      % node n1
TN(:,3)=(R(2)+R(3))/2*((2/(R(2)+R(3))-1/(R(2)/2+R(3)+R(4)/2)).*TN(:,2)+(1/(R(2)/2+R(3)+R(4)/2)).*TN(:,4));  % node n3
TN(:,5)=R(5)/2*((2/R(5)-1/(R(4)/2+R(5))).*T0*ones(length(tt),1)+(1/(R(4)/2+R(5))).*TN(:,4));                % node n5

% Causal model results presentation
figure
plot(tt,Ti(T0,Tf,ti,tt),'-.','LineWidth',0.5)
hold on
plot(tt,TN)
grid minor
xlabel('$t\, [s]$')
ylabel('$T\, [K]$')
title('Causal Model, ode45')
legend('$T_i$','$T_1$','$T_2$','$T_3$','$T_4$','$T_5$','Location','east')

% 1N acausal model results presentation
figure
plot(tt,TS1)
grid minor
xlabel('$t\, [s]$')
ylabel('$T\, [K]$')
title('Acausal Model, ode15s')
legend('$T_1$','$T_2$','$T_3$','$T_4$','$T_5$','Location','east')

% Causal-1N-acausal model results comparison
figure
semilogy(tt,abs(TN-TS1))
grid minor
xlabel('$t\, [s]$')
ylabel('$|T_{i,c}-T_{i,a}|\quad [K]$')
legend('$T_1$','$T_2$','$T_3$','$T_4$','$T_5$','Location','southeast')
title('Acausal-Causal Model Error')

% Acausal 1N-2N model results presentation
figure
plot(tt,TS1,'b',tt,TS2,'r')
grid minor
xlabel('$t\, [s]$')
ylabel('$T\, [K]$')
title('1-node, 2-nodes Acausal Models Comparison')
legend('$T_{i,1n}$','','','','','$T_{i,2n}$','Location','northwest')

% Acausal 1N-2N model results comparison
figure
plot(tt,abs(TS1-TS2))
grid minor
xlabel('$t\, [s]$')
ylabel('$|T_{1n}-T_{2n}|\quad [K]$')
title('1-node, 2-nodes Acausal Models Deviation')
legend('$T_1$','$T_2$','$T_3$','$T_4$','$T_5$','Location','northeast')

% Used defined graphical settings removed
set(0,'DefaultAxesFontSize','remove');
set(0,'DefaultLegendFontSize','remove');
set(0,'DefaultLineLineWidth','remove');
set(groot,'defaultTextInterpreter','remove');
set(groot,'defaultLegendInterpreter','remove');

%% Functions

function [T] = Ti(T0,Tf,ti,t)
%     Computes internal wall time dependent temperature
%     Example: [T] = Ti(T0, Tf, ti, t)
%     INPUTS:
%         T0  [1x1] initial temperature [K]
%         Tf  [1x1] final temperature [K]
%         ti  [1x3] event time instants [s]
%         t   [1x1] time independent variable [s]
%     OUTPUTS:
%         T   [1x1] internal wall temperature at t

    if ~issorted(ti)
        error('Inconsistent time events')
    elseif Tf<=T0
        error('Inconsistent boundary conditions')
    end
       
    T=Tf.*(t>ti(2))+(T0+(Tf-T0)*t/ti(2)).*(t<=ti(2));
end

