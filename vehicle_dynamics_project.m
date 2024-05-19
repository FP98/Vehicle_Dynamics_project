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
beta = zeros(n,1);

% Forward integration
v1_f = zeros(n,1);
v2_f = zeros(n,1);
v_f = zeros(n,1);
xg_f = zeros(n,1);
yg_f = zeros(n,1);
yaw_f = zeros(n,1);

for i=1:n-1
    v1_f(i) = tan(beta1(i))*speed(i) - omega_z(i)*a1;
    v2_f(i) = tan(beta2(i))*speed(i) + omega_z(i)*a2;

    v_f(i) = 0.5*( v1_f(i) + v2_f(i) );

    xg_f(i+1) = xg_f(i) - ( speed(i)*cos( yaw_f(i) ) - v_f(i)*sin( yaw_f(i) ) )*dt;
    yg_f(i+1) = yg_f(i) - ( speed(i)*sin( yaw_f(i) ) + v_f(i)*cos( yaw_f(i) ) )*dt;

    yaw_f(i+1) = yaw_f(i) + 1/2*( omega_z(i+1) + omega_z(i) )*dt;
    yaw_f(i+1) = atan2( sin( yaw_f(i+1) ), cos( yaw_f(i+1) ) );
    

end

% Backward integration
v1_i = zeros(n,1);
v2_i = zeros(n,1);
v_i = zeros(n,1);
xg_i = zeros(n,1);
yg_i = zeros(n,1);
yaw_i = zeros(n,1);

for i=n:-1:2
    v1_i(i) = tan(beta1(i))*speed(i) - omega_z(i)*a1;
    v2_i(i) = tan(beta2(i))*speed(i) + omega_z(i)*a2;

    v_i(i) = 0.5*( v1_i(i) + v2_i(i) );

    xg_i(i-1) = xg_i(i) + ( speed(i)*cos( yaw_i(i) ) - v_i(i)*sin( yaw_i(i) ) )*dt;
    yg_i(i-1) = yg_i(i) + ( speed(i)*sin( yaw_i(i) ) + v_i(i)*cos( yaw_i(i) ) )*dt;

    yaw_i(i-1) = yaw_i(i) - ( omega_z(i) )*dt;
    yaw_i(i-1) = atan2( sin( yaw_i(i-1) ), cos( yaw_i(i-1) ) );
    
end

% Defining weight vectors

wf = linspace(1,0,n)';
wi = linspace(0,1,n)';

v = wf.*v_f + wi.*v_i;
xg = wf.*xg_f + wi.*xg_i;
yg = wf.*yg_f + wi.*yg_i;
c_yaw = wf.*cos(yaw_f) + wi.*cos(yaw_i);
s_yaw = wf.*sin(yaw_f) + wi.*sin(yaw_i);
yaw = atan2(s_yaw, c_yaw);



% Plotting cg trajectory
figure
title("Center of mass trajectory");
plot(xg, yg, 'b')
hold on
quiver(xg(1), yg(1), -100*cos( yaw(1) ), -100*sin( yaw(1) ), 'MaxHeadSize', 20, 'LineWidth', 2, 'Color', 'r');
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

% Searching the minimum ax
ax_min = 0;                     % minimum longitudinal acceleration
n_min = 0;                      % sample to wich we have ax_min
for i=1:n
    if ax(i) < ax_min
        ax_min = ax(i);
        n_min = i;
    end
end

% Grip estimation (mu)
mu = ( abs(ax_min) - ( Xa(n_min)/massa_vettura(n_min) ) ) / ( g + ( Za1(n_min) + Za2(n_min) )/massa_vettura(n_min) );

%% Evalueting lateral forces Y1 Y2

% Using single track model
Y1 = zeros(n,1);
Y2 = zeros(n,1);

for i = 1:n
    Y1(i) = ( d_omega_z_f(i)*Jzz(1) + massa_vettura(i)*ay(i) ) / passo_vettura(1);
    Y2(i) = massa_vettura(i)*ay(i) - Y1(i);
end

% Plotting lateral forces Y1 Y2
figure
title("Lateral forces");
% Front lateral force (Y1)
subplot(1,2,1)
plot(time, Y1, 'b')
title("Front lateral force (Y1)")
xlabel("[s]")
ylabel("[N]")
grid on
hold off
% Rear lateral force (Y2)
subplot(1,2,2)
plot(time, Y2, 'b')
title("Rear lateral force (Y2)")
xlabel("[s]")
ylabel("[N]")
grid on
hold off

%% Evaluating yaw moment N

% Using single track model
N = zeros(n,1);
for i = 1:n
    N(i) = d_omega_z_f(i)*Jzz(1);
end

% Plotting yaw moment
figure
plot(time, N, 'b')
title("Yaw moment (N)");
xlabel("[s]")
ylabel("[Nm]")
grid on
hold off

%% Power limited curve 
% Plotting u vs ax
figure
dot_dim = 10;
scatter(speed, ax, dot_dim, 'filled')
title("Power limited curve");
xlabel("vx [g]")
ylabel("ax [g]")
grid on
hold off

%% Grip limited curve
Fy = Y1 + Y2;
% Plotting u vs Fy
figure
scatter(speed, Fy, dot_dim, 'filled')
title("Grip limited curve");
xlabel("u [m/s]")
ylabel("Fy [N]")
grid on
hold off

%% gg plot
% Plotting ax vs ay
figure
scatter(ax/g, ay/g, dot_dim, 'filled')
title("g-g plot");
xlabel("ax [g]")
ylabel("ay [g]")
grid on
hold off

%% Fixed certrodes

v = smoothdata(v, "loess","SmoothingFactor",0.25);
c_yaw = smoothdata(c_yaw, "loess","SmoothingFactor",0.25);
s_yaw = smoothdata(s_yaw, "loess","SmoothingFactor",0.25);
yaw = atan2(s_yaw, c_yaw);
R = speed./omega_z_f;
S = -v./omega_z_f;
xc = zeros(n,1);
yc = zeros(n,1);

for i=1:n
    if abs( R(i) ) < 30*abs( S(i) )
        GC_v = [S(i); R(i)];
        GC_f = [-cos( yaw(i) ) sin( yaw(i) ); -sin( yaw(i) ) -cos( yaw(i) )]*GC_v;
        OC_f = GC_f + [xg(i); yg(i)];
        xc(i) = OC_f(1);
        yc(i) = OC_f(2);
    end
end

% Finding curves
n_s = 0;
n_e = 0;
j = 1;
k = 1;

if xc(1) ~= 0
    n_s(j) = 1;
    j = j + 1;
else
    for i=2:n
        if (xc(i) ~= 0) && (xc(i-1) == 0)
            n_s(j) = i;
            j = j+1;
        else if (xc(i) == 0) && (xc(i-1) ~= 0)
                n_e(k) = i-1;
                k = k+1;
        end 
        end
    end
end

% Plotting curves and relative fixed centrodes
figure
k = 1;
for j=1:size(n_s,2)
    if n_e(j) - n_s(j) > 50
        subplot(2,7,k)
        plot(xc(n_s(j):n_e(j)),yc(n_s(j):n_e(j)), 'r', 'LineWidth',2)
        hold on
        plot(xg(n_s(j):n_e(j)), yg(n_s(j):n_e(j)), 'b')
        title(['Curve ' num2str(k)])
        legend("Fixed centrode","Curve");
        grid on
        axis equal
        hold off
        if k == 8
            n_s_c8 = n_s(j);
            n_e_c8 = n_e(j);
        end
        k = k+1;
    end
end

%% Moving centrode
n_c8 = (n_s_c8:1:n_e_c8)'; % Sample of curve 8
n1= n_c8(1); 
n2 = n_c8(floor(length(n_c8)/2));
n3 = n_c8(end);
n_c = [n1;n2;n3];

xm = zeros( length(n_c8), 3);
ym = zeros( length(n_c8), 3);
for j=1:3
    for i=1:length(n_c8)
        xm(i,j) = xg(n_c(j)) - S( i + n1 - 1 )*cos( yaw( n_c(j) ) ) + R( i + n1 - 1 )*sin( yaw( n_c(j) ) );
        ym(i,j) = yg(n_c(j)) - S( i + n1 - 1 )*sin( yaw( n_c(j) ) ) - R( i + n1 - 1 )*cos( yaw( n_c(j) ) );
    end
end

% Plotting
figure 
plot(xg(n1:n3), yg(n1:n3))
hold on
plot(xc(n1:n3),yc(n1:n3),'LineWidth',2)
for j=1:3
    plot(xm(:,j), ym(:,j), 'LineWidth',2)
end
axis equal
grid on
hold off

        


