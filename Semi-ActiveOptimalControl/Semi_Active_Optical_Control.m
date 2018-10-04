%********************************%
%   Semi-Active Suspension 
%   > Optimal Control
%   > Quarter-car model
%   > Sinwave Input
%********************************%

clear; close all;

%% Switch of linear or non-linear damper model
% 0 : Non-liner
% 1 : Linear
SW = 1;

%% Car Parameter
m2   = 240;    % body mass                      [kg]
m1   = 36;     % Tire mass                      [kg]
k2   = 16000;  % Suspension stiffness           [N/m]
k1   = 160000; % Tire stiffness                 [N/m]
c2   = 1000;   % Suspension damping             [Nsec/m]
Cmax = 3000;   % Maximum semi-active dampimg    [Nsec/m]

%% Simulatino Parameter
fr   = 1;              % Road input frequency [Hz]
q    = [400 16 400 16]; % Weighting factors
Tend = 3.0;             % Simulation time [sec]
Ts   = 0.001;           % Sampling time [sec]

%% Equation of State
% x_dot = Ax + Bu + Wz0
% <State Quantity : x>
%   x1 : Suspension deflection 
%   x2 : Absolute velocity of sprung mass
%   x3 : Tire deflection
%   x4 : Absolute velocity of unsprung mass
A  = [      0      1      0     -1;
       -k2/m2 -c2/m2      0  c2/m2;
            0      0      0      1;
        k2/m1  c2/m1 -k1/m1 -c2/m1];

% Using nonlinear damper model
An = [      0      1      0     -1;
       -k2/m2      0      0      0;
            0      0      0      1;
        k2/m1      0 -k1/m1      0];
  
B0 = [0 1/m2 0 -1/m1]';

W  = [0 0 -1 0]';

R0 = 1/(m2)^2;

S0 = [-k2/(m2)^2 -c2/(m2)^2 0 c2/(m2)^2];

Q0 = zeros(4,4);
Q0(1,1) = (k2/m2)^2+q(1);
Q0(2,2) = (c2/m2)^2+q(2);
Q0(3,3) = q(3);
Q0(4,4) = (c2/m2)^2+q(4);
Q0(1,2) = c2*k2/(m2)^2;
Q0(1,4) = -c2*k2/(m2)^2;
Q0(2,4) = -c2^2/(m2)^2;
Q0(2,1) = Q0(1,2);
Q0(4,1) = Q0(1,4);
Q0(4,2) = Q0(2,4);

% Output equation
% y = Cx + Du + Ez0
%   y1 : Suspension deflection 
%   y2 : Absolute velocity of sprung mass
%   y3 : Tire deflection
%   y4 : Absolute velocity of unsprung mass
%   y5 : Sprung Vertical Acceleration
%   y6 : Sprung Vertical Displacement
C = [eye(4);A(2,:); 1 0 1 0];
D = [0 0 0 0 B0(2) 0]';
E = [0 0 0 0 0 1]';

    
%% Riccati Equation
A_bar = A - B0*R0^-1*S0;
Q_bar = Q0 -S0'*R0^-1*S0;

[P,L,G] = care(A_bar, B0, Q_bar, R0);

%% Simulation
sim('Semi_Active_Quarter_Model');

%% Plot
Time = tout;

figure(1);
for i = 1:4
    sbp1(i) = subplot(2,2,i);
    xlabel(sbp1(i),{'Time [sec]'});
    xlim([0 Time(end)]);
    grid on;hold on;
end
% Input
plot(sbp1(1),Time, Input, 'k');
% Passive
plot(sbp1(1),Time, Output_Passive(:, 6), 'r');
plot(sbp1(2),Time, Output_Passive(:, 5), 'r');
plot(sbp1(3),Time, Output_Passive(:, 1), 'r');
plot(sbp1(4),Time, Output_Passive(:, 3), 'r');
% Semi-Active F/B Optimal Control (LQR)
plot(sbp1(1),Time, Output_Semi(:, 6), 'b');
plot(sbp1(2),Time, Output_Semi(:, 5), 'b');
plot(sbp1(3),Time, Output_Semi(:, 1), 'b');
plot(sbp1(4),Time, Output_Semi(:, 3), 'b');

ylabel(sbp1(1),{'Sprung Vertical', 'Displacement [m]'});
ylabel(sbp1(2),{'Sprung Vertical', 'Acceleration [m/sec^2]'});
ylabel(sbp1(3),{'Suspension Deflection [m]'});
ylabel(sbp1(4),{'Tire Deflection [m]'});

legend(sbp1(1),{'Input','Passive', 'Semi-active LQR'});

