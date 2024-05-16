% Load telemetry data from excell
clear all
close all
clc

% Defining file path
file_path = 'telemetrie_2012_per_2023.xls';

% Defining the variables where the data are saved
[num_data] = xlsread(file_path);

time = num_data(:,1);       % [time] = s
dist = num_data(:,2);       % [dist] = m
speed = num_data(:,3);      % [speed] = km/h
ax = num_data(:,4);         % [ax] = g
ay = num_data(:,5);         % [ay] = g
airspeed = num_data(:,6);   % [airspeed] = km/h
sideslip_front = num_data(:,7);     % [sideslip_front] = deg
sideslip_rear =  num_data(:,8);     % [sideslip_rear] = deg
omega_z = num_data(:,9);    % [omega_z_vettura] = deg/s 
massa_vettura = num_data(:,10);     % [massa_vettura] =
farf = num_data(:,11);
steer = num_data(:,12);
p_brake = num_data(:,13);
carreggiata_front = num_data(:,14);
carreggiata_rear = num_data(:,15);
weight_dist = num_data(:,16);
Jxx = num_data(:,17);
Jyy = num_data(:,18);
Jzz = num_data(:,19);
Jzx = num_data(:,20);
passo_vettura = num_data(:,21);
cz_front = num_data(:,22);
cz_rear = num_data(:,23);
air_dens = num_data(:,24);
cx_tot = num_data(:,25);

dt = time(2) - time(1);
g = 9.81;                       % Accelerazione di gravità [g] = m/s^2

% Converting in SI units:

speed = speed * 1e3/3600;
airspeed = airspeed * 1e3/3600;
ax = ax * g;
ay = ay * g;
omega_z = - omega_z * pi/180;

sideslip_front = - sideslip_front*pi/180;
sideslip_rear =  - sideslip_rear*pi/180;
steer = - steer*pi/180;

n = size(time,1);


grid on
hold on
axis equal


 

%% 5) Filtering and Differentiation of yaw rate

omega_z_f = smoothdata(omega_z, ...
    "loess","SmoothingFactor",0.25);

d_omega_z_f = zeros(n,1);
for i=1:n-1
    d_omega_z_f(i) =  ( omega_z_f(i+1) - omega_z_f(i) ) / dt;
end
d_omega_z_f(n) = d_omega_z_f(n-1);

%% 4) Reconstruction of the trajectory of center of mass 

% By center of velocity
beta1 = sideslip_front;
beta2 = sideslip_rear;
a1 = weight_dist(1)*passo_vettura(1);        % semipasso anteriore
a2 = passo_vettura(1) - a1;                   % semipasso posteriore
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
    

    % beta(i) = atan2(v(i), u(i));
    % 
    % if  ( beta(i) > beta1(i) ) || ( beta(i) < beta2(i) )
    %     beta(i) = 0.5*( beta1(i) + beta2(i) );
    %     k = k+1;
    % end
    % 
    % Finding xg yg
    % u_ = speed(i)*cos(beta(i));
    % v_ = speed(i)*sin(beta(i));

    % speed_s(i) = sqrt( u_^2 + v_^2); 

    xg_f(i+1) = xg_f(i) + (  speed(i)*cos( yaw(i) ) - v(i)*sin( yaw(i) ) )*dt;
    yg_f(i+1) = yg_f(i) + ( speed(i)*sin( yaw(i) ) + v(i)*cos( yaw(i) ) )*dt;

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

    xg_i(i-1) = xg_i(i) - ( speed(i)*cos( yaw(i) ) - v(i)*sin( yaw(i) ) )*dt;
    yg_i(i-1) = yg_i(i) - ( speed(i)*sin( yaw(i) ) + v(i)*cos( yaw(i) ) )*dt;

    yaw(i-1) = yaw(i) - ( omega_z(i) )*dt;
    yaw(i-1) = atan2( sin( yaw(i-1) ), cos( yaw(i-1) ) );
    
end

% Defining weight vectors

wf = linspace(1,0,n)';
wi = linspace(0,1,n)';

xg = wf.*xg_f + wi.*xg_i;
yg = wf.*yg_f + wi.*yg_i;

x = -xg;
y = -yg;
