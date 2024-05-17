% Vehicle Dynamics project
clear 
close all
clc

%% Loading data from excel and converting in SI units
% Defining file path
file_path = 'telemetrie_2012_per_2023.xls';

% Defining the variables where the data are saved
[num_data] = xlsread(file_path);

time = num_data(:,1);                   % [time] = s
dist = num_data(:,2);                   % [dist] = m
speed = num_data(:,3);                  % [speed] = km/h
ax = num_data(:,4);                     % [ax] = m/s^2/g
ay = num_data(:,5);                     % [ay] = m/s^2/g
airspeed = num_data(:,6);               % [airspeed] = km/h
sideslip_front = num_data(:,7);         % [sideslip_front] = deg
sideslip_rear =  num_data(:,8);         % [sideslip_rear] = deg
omega_z = num_data(:,9);                % [omega_z_vettura] = deg/s 
massa_vettura = num_data(:,10);         % [massa_vettura] =
farf = num_data(:,11);
steer = num_data(:,12);                 % [steer] = deg
p_brake = num_data(:,13);
carreggiata_front = num_data(:,14);
carreggiata_rear = num_data(:,15);
weight_dist = num_data(:,16);
Jxx = num_data(:,17);                   % [Jxx] = kg*m^2
Jyy = num_data(:,18);                   % [Jyy] = kg*m^2
Jzz = num_data(:,19);                   % [Jzz] = kg*m^2
Jzx = num_data(:,20);                   % [Jzx] = kg*m^2
passo_vettura = num_data(:,21);         % [passo_vettura] = m
cz_front = num_data(:,22);
cz_rear = num_data(:,23);
air_dens = num_data(:,24);              % [air_dens] = kg/m^3
cx_tot = num_data(:,25);

dt = time(2) - time(1);
g = 9.81;                               % gravity acceleration, [g] = m/s^2

% Converting in SI units:

speed = speed * 1e3/3600;
airspeed = airspeed * 1e3/3600;
ax = ax * g;
ay = ay * g;
omega_z = - omega_z * pi/180;                % Fliping the sign of angles an angles rates
                                             % to use vehicle frame like the one used in the course
sideslip_front = sideslip_front*pi/180;    
sideslip_rear = sideslip_rear*pi/180;
steer = steer*pi/180;

%% Plotting signals

% Plotting speed
figure
title("Speed");
% longitudinal speed of car (u)
subplot(1,2,1)
plot(time, speed, 'b')
title("Longitudinal speed of vehicle (u)")
xlabel("[s]")
ylabel("[m/s]")
grid on
hold off
% Relative speed of the air flow (Va)
subplot(1,2,2)
plot(time, airspeed, 'b')
title("Relative speed of the airflow (V_a)")
xlabel("[s]")
ylabel("[m/s]")
grid on
hold off

% Plotting accelerations
figure
title("Acceleration");
% Acceleration along the longitudinal axes of vehicle frame (ax)
subplot(1,2,1)
plot(time, ax, 'b')
title("Longitudinal acceleration (a_x)")
xlabel("[s]")
ylabel("[m/s^2]")
grid on
hold off
% Acceleration along the lateral axes of vehicle frame (ax)
subplot(1,2,2)
plot(time, ay, 'b')
title("Lateral acceleration (a_y)")
xlabel("[s]")
ylabel("[m/s^2]")
grid on
hold off

% Plotting angles
figure
title("Angles");
% Front sideslip angle (beta1)
subplot(1,3,1)
plot(time, sideslip_front, 'b')
title("Front sideslip angle (\beta_1)")
xlabel("[s]")
ylabel("[rad]")
grid on
hold off
% Rear sideslip angle (beta2)
subplot(1,3,2)
plot(time, sideslip_rear, 'b')
title("Rear sideslip angle (\beta_2)")
xlabel("[s]")
ylabel("[rad]")
grid on
hold off
% Steer angle (\delta)
subplot(1,3,3)
plot(time, steer, 'b')
title("Steer angle (\delta)")
xlabel("[s]")
ylabel("[rad]")
grid on
hold off

%% 4) Reconstruction of the trajectory of center of mass 

% By center of velocity
n = size(time,1);

beta1 = sideslip_front;
beta2 = sideslip_rear;
a2 = weight_dist(1)*passo_vettura(1);        % semipasso anteriore
a1 = passo_vettura(1) - a2;                   % semipasso posteriore
yaw = zeros(n,1);
beta = zeros(n,1);

v1 = zeros(n,1);
v2 = zeros(n,1);
v = zeros(n,1);


speed_s = [speed(1); zeros(n-1,1)];

dist_s = 0;

xg_f = zeros(n,1);
yg_f = zeros(n,1);
k = 0;
for i=1:n-1

    % Computing v u beta
    v1(i) = tan(beta1(i))*speed(i) - omega_z(i)*a1;
    v2(i) = tan(beta2(i))*speed(i) + omega_z(i)*a2;

    v(i) = 0.5*( v1(i) + v2(i) );

    xg_f(i+1) = xg_f(i) - ( speed(i)*cos( yaw(i) ) - v(i)*sin( yaw(i) ) )*dt;
    yg_f(i+1) = yg_f(i) - ( speed(i)*sin( yaw(i) ) + v(i)*cos( yaw(i) ) )*dt;

    yaw(i+1) = yaw(i) + 1/2*( omega_z(i+1) + omega_z(i) )*dt;
    yaw(i+1) = atan2( sin( yaw(i+1) ), cos( yaw(i+1) ) );
    dist_s = dist_s + sqrt( ( xg_f(i+1) - xg_f(i) )^2 + ( yg_f(i+1) - yg_f(i) )^2 );

end

xg_i = [zeros(n,1)];
yg_i = [zeros(n,1)];
yaw(end) = 0;
for i=n:-1:2
    v1(i) = tan(beta1(i))*speed(i) - omega_z(i)*a1;
    v2(i) = tan(beta2(i))*speed(i) + omega_z(i)*a2;

    v(i) = 0.5*( v1(i) + v2(i) );

    xg_i(i-1) = xg_i(i) + ( speed(i)*cos( yaw(i) ) - v(i)*sin( yaw(i) ) )*dt;
    yg_i(i-1) = yg_i(i) + ( speed(i)*sin( yaw(i) ) + v(i)*cos( yaw(i) ) )*dt;

    yaw(i-1) = yaw(i) - ( omega_z(i) )*dt;
    yaw(i-1) = atan2( sin( yaw(i-1) ), cos( yaw(i-1) ) );
    
end

% Defining weight vectors

wf = linspace(1,0,n)';
wi = linspace(0,1,n)';

xg = wf.*xg_f + wi.*xg_i;
yg = wf.*yg_f + wi.*yg_i;

% Plotting cg trajectory
figure
title("Center of mass trajectory");
plot(xg, yg, 'b')
xlabel("[m]")
ylabel("[m]")
grid on
axis equal
hold off

%% Filtering and Differentiation of yaw rate

% Filtering omega_z
omega_z_f = smoothdata(omega_z, "loess","SmoothingFactor",0.25);

% Differentiating omega_z
d_omega_z_f = zeros(n,1);
for i=1:n-1
    d_omega_z_f(i) =  ( omega_z_f(i+1) - omega_z_f(i) ) / dt;
end
d_omega_z_f(n) = d_omega_z_f(n-1);


% Plotting filtered and differentiated data
figure
title("\omega_z and d\omega_z");
% Real vs Filtered omega_z
subplot(1,2,1)
plot(time, omega_z, 'r')
hold on
plot(time, omega_z_f, 'b','LineWidth',2)
title("Real \omega_z vs filtered \omega_z")
xlabel("[s]")
ylabel("[rad/s]")
legend("Real","Filtered");
grid on
hold off
% Differentiated omega_z (d_omega_z)
subplot(1,2,2)
plot(time, d_omega_z_f, 'b')
title("Differentiation of filtered \omega_z (d\omega_z)")
xlabel("[s]")
ylabel("[rad/s^2]")
grid on
hold off

%% Aerodynamic loads estimation
ro = air_dens(1);
Xa = zeros(n,1);
Za1 = zeros(n,1);
Za2 = zeros(n,1);

for i=1:n
    Xa(i) = 0.5*ro*airspeed(i)^2*cx_tot(i);
    Za1(i) = 0.5*ro*airspeed(i)^2*cz_front(i);
    Za2(i) = 0.5*ro*airspeed(i)^2*cz_rear(i);
end

% Plotting aerodynamics forces
figure
title("Aerodinamics forces");
% Drag force (Xa)
subplot(1,3,1)
plot(time, Xa, 'b')
title("Drag force (X_a)")
xlabel("[s]")
ylabel("[N]")
grid on
hold off
% Front down force (Za1)
subplot(1,3,2)
plot(time, Za1, 'b')
title("Front down force (Z_a_1)")
xlabel("[s]")
ylabel("[N]")
grid on
hold off
% Rear down force (Za2)
subplot(1,3,3)
plot(time, Za2, 'b')
title("Rear down force (Z_a_2)")
xlabel("[s]")
ylabel("[N]")
grid on
hold off

%% Physical grip estimation




