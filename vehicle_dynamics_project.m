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
massa_vettura = num_data(:,10);         % [massa_vettura] = kg
farf = num_data(:,11);                  % [farf] = %
steer = num_data(:,12);                 % [steer] = deg
p_brake = num_data(:,13);               % [p_brake] = psi
carreggiata_front = num_data(:,14);     % [carreggiata_front] = m
carreggiata_rear = num_data(:,15);      % [carreggiata_rear] = m
weight_dist = num_data(:,16);           % [weight_dist] = fraction
Jxx = num_data(:,17);                   % [Jxx] = kg*m^2
Jyy = num_data(:,18);                   % [Jyy] = kg*m^2
Jzz = num_data(:,19);                   % [Jzz] = kg*m^2
Jzx = num_data(:,20);                   % [Jzx] = kg*m^2
passo_vettura = num_data(:,21);         % [passo_vettura] = m
cz_front = num_data(:,22);              % [cz_front] = m^2
cz_rear = num_data(:,23);               % [cz_rear] = m^2
air_dens = num_data(:,24);              % [air_dens] = kg/m^3
cx_tot = num_data(:,25);                % [cx_tot] = m^2

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
p_brake = p_brake*6894.76;

%% Plotting signals

% Plotting speed
figure
% longitudinal speed of car (u)
plot(time, speed, 'b')
title("Longitudinal speed of vehicle (u)")
xlabel("[s]")
ylabel("[m/s]")
grid on
hold off
% Relative speed of the air flow (Va)
figure
plot(time, airspeed, 'b')
title("Relative speed of the airflow (V_a)")
xlabel("[s]")
ylabel("[m/s]")
grid on
hold off

% Plotting accelerations
figure
% Acceleration along the longitudinal axes of vehicle frame (ax)
plot(time, ax, 'b')
title("Longitudinal acceleration (a_x)")
xlabel("[s]")
ylabel("[m/s^2]")
grid on
hold off
% Acceleration along the lateral axes of vehicle frame (ax)
figure
plot(time, ay, 'b')
title("Lateral acceleration (a_y)")
xlabel("[s]")
ylabel("[m/s^2]")
grid on
hold off

% Plotting angles
figure

% Front sideslip angle (alpha1)

plot(time, sideslip_front, 'b')
title("Front sideslip angle (\alpha_f)")
xlabel("[s]")
ylabel("[rad]")
grid on
hold off
% Rear sideslip angle (alpha2)
figure
plot(time, sideslip_rear, 'b')
title("Rear sideslip angle (\alpha_r)")
xlabel("[s]")
ylabel("[rad]")
grid on
hold off
% Steer angle (\delta)
figure
plot(time, steer, 'b')
title("Steer angle (\delta)")
xlabel("[s]")
ylabel("[rad]")
grid on
hold off

% Plotting brake pressure and farf
figure
% p_brake

plot(time, p_brake,'b')
title("Pressure of the brakes")
xlabel("[s]")
ylabel("[Pa x 10^5]")
grid on
hold off
% farf
figure
plot(time, farf,'b')
title("Opening of the motor valve")
xlabel("[s]")
ylabel("[%]")
grid on
hold off


%% 4) Reconstruction of the trajectory of center of mass 

% By center of velocity
n = size(time,1);

beta_f = steer - sideslip_front;
beta_r = -sideslip_rear;
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
    v1_f(i) = tan(beta_f(i))*speed(i) - omega_z(i)*a1;
    v2_f(i) = tan(beta_r(i))*speed(i) + omega_z(i)*a2;

    v_f(i) = 0.5*( v1_f(i) + v2_f(i) );

    xg_f(i+1) = xg_f(i) - ( speed(i)*cos( yaw_f(i) ) - v_f(i)*sin( yaw_f(i) ) )*dt;
    yg_f(i+1) = yg_f(i) - ( speed(i)*sin( yaw_f(i) ) + v_f(i)*cos( yaw_f(i) ) )*dt;

    yaw_f(i+1) = yaw_f(i) + 1/2*( omega_z(i+1) + omega_z(i) )*dt;
    yaw_f(i+1) = atan2( sin( yaw_f(i+1) ), cos( yaw_f(i+1) ) );
end

% Backward integration
v1_b = zeros(n,1);
v2_b = zeros(n,1);
v_b = zeros(n,1);
xg_b = zeros(n,1);
yg_b = zeros(n,1);
yaw_b = zeros(n,1);

for i=n:-1:2
    v1_b(i) = tan(beta_f(i))*speed(i) - omega_z(i)*a1;
    v2_b(i) = tan(beta_r(i))*speed(i) + omega_z(i)*a2;

    v_b(i) = 0.5*( v1_b(i) + v2_b(i) );

    xg_b(i-1) = xg_b(i) + ( speed(i)*cos( yaw_b(i) ) - v_b(i)*sin( yaw_b(i) ) )*dt;
    yg_b(i-1) = yg_b(i) + ( speed(i)*sin( yaw_b(i) ) + v_b(i)*cos( yaw_b(i) ) )*dt;

    yaw_b(i-1) = yaw_b(i) - ( omega_z(i) )*dt;
    yaw_b(i-1) = atan2( sin( yaw_b(i-1) ), cos( yaw_b(i-1) ) );
end

% Defining weight vectors
wf = linspace(1,0,n)';          % Forward weight vector
wb = linspace(0,1,n)';          % Backward weight vector

% G trajectory
xg = wf.*xg_f + wb.*xg_b;
yg = wf.*yg_f + wb.*yg_b;

c_yaw = wf.*cos(yaw_f) + wb.*cos(yaw_b);
s_yaw = wf.*sin(yaw_f) + wb.*sin(yaw_b);
yaw = atan2(s_yaw, c_yaw);
v = wf.*v_f + wb.*v_b;



% Plotting cg trajectory
figure
plot(xg, yg, 'b')
title("Center of mass trajectory");
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
% Real vs Filtered omega_z
plot(time, omega_z, 'r')
hold on
plot(time, omega_z_f, 'b','LineWidth',2)
title("Real r vs filtered r")
xlabel("[s]")
ylabel("[rad/s]")
legend("Real","Filtered");
grid on
hold off
% Differentiated omega_z (d_omega_z)
figure
plot(time, d_omega_z_f, 'b')
title("Differentiation of filtered r ($\dot{r}$)", "Interpreter", "latex")
xlabel("[s]")
ylabel("[rad/s^2]")
grid on
hold off

%% Aerodynamic loads estimation
rho = air_dens(1);
Xa = zeros(n,1);
Za1 = zeros(n,1);
Za2 = zeros(n,1);

for i=1:n
    Xa(i) = 0.5*rho*airspeed(i)^2*cx_tot(i);
    Za1(i) = 0.5*rho*airspeed(i)^2*cz_front(i);
    Za2(i) = 0.5*rho*airspeed(i)^2*cz_rear(i);
end

% Plotting aerodynamics forces
figure
% Drag force (Xa)
plot(time, Xa, 'b')
title("Drag force (X_a)")
xlabel("[s]")
ylabel("[N]")
grid on
hold off
% Front down force (Za1) & Rear down force (Za2)
figure
plot(time, Za1, 'b')
title("Front down force (Z_a_1) and rear down force (Z_a_2)")
xlabel("[s]")
ylabel("[N]")
grid on
hold on
plot(time, Za2, 'r')
legend("Front down force (Z_a_1)","Rear down force (Z_a_2)");
hold off

%% Physical grip estimation

% Minimum ax search
ax_min = 0;                     % minimum longitudinal acceleration
n_min = 0;                      % sample to wich we have ax_min
for i=1:n
    if ax(i) < ax_min
        ax_min = ax(i);
        n_min = i;
    end
end

% Grip estimation (mu)
mu = ( abs(ax_min) - ( Xa(n_min)/massa_vettura(n_min) ) ) / ( g + ( Za1(n_min) ...
    + Za2(n_min) )/massa_vettura(n_min) );

%% Evalueting lateral forces Y1 Y2

% Evaluating Y1 and Y2 using single track model
Y1 = zeros(n,1);
Y2 = zeros(n,1);

for i = 1:n
    Y1(i) = ( d_omega_z_f(i)*Jzz(1) + massa_vettura(i)*ay(i)*a2 ) / passo_vettura(1);
    Y2(i) = massa_vettura(i)*ay(i) - Y1(i);
end

% Evaluating Y1 and Y2 using double track model and rigid wheels
Y1_d = zeros(n,1);
Y2_d = zeros(n,1);
X21 = zeros(n,1);
X22 = zeros(n,1);
eta = 0.5;                  % Differential efficency
Rr = 0.36;                  % Rolling radius of the wheels
chi = 1000;                 % Time constant
t2 = carreggiata_rear(1);

for i = 1:n
    u21 = speed(i) - omega_z(i)*t2/2;
    u22 = speed(i) + omega_z(i)*t2/2;

    omega21 = u21/Rr;
    omega22 = u22/Rr;

    delta_omega = omega22 - omega21;

    zeta = atan(chi*delta_omega)/pi/2;

    X2 = massa_vettura(i)*ax(i) + Xa(i);

    X21(i) = X2 / ( eta^zeta +1 );
    X22(i) = X21(i) * eta^zeta;

    delta_X2 = X22(i) - X21(i);

    Y1_d(i) = ( d_omega_z_f(i)*Jzz(1) + massa_vettura(i)*ay(i)*a2 - delta_X2*t2 ) / passo_vettura(1);
    Y2_d(i) = massa_vettura(i)*ay(i) - Y1_d(i);
end



% Plotting lateral forces Y1 Y2
figure
% Front lateral force (Y1)
plot(time, Y1, 'b')
hold on
plot(time, Y1_d, 'r')
title("Front lateral force (Y1)")
xlabel("[s]")
ylabel("[N x 10^4]")
legend("Y1 single track model", "Y1 double track model")
grid on
hold off
% Rear lateral force (Y2)
figure
plot(time, Y2, 'b')
hold on
plot(time, Y2_d, 'r')
title("Rear lateral force (Y2)")
xlabel("[s]")
ylabel("[N x 10^4]")
legend("Y2 single track model", "Y2 double track model")
grid on
hold off

figure 
plot(time, X21, 'g')
hold on
plot(time, X22,'k')
hold off


% Evaluating yaw moment N
N = zeros(n,1);
for i = 1:n
    N(i) = d_omega_z_f(i)*Jzz(1);
end

% Plotting yaw moment
figure
plot(time, N, 'b')
title("Yaw moment (Nm)");
xlabel("[s]")
ylabel("[Nm]")
grid on
hold off

%% gg plot
% Plotting ax vs ay
dot_dim = 10;
figure
scatter(ax/g, ay/g, dot_dim, 'filled')
title("g-g plot");
xlabel("ax [g]")
ylabel("ay [g]")
grid on
hold off

%% Fixed certrodes

% Filtering data
speed = smoothdata(speed, "loess","SmoothingFactor",0.25);
v = smoothdata(v, "loess","SmoothingFactor",0.25);
c_yaw = smoothdata(c_yaw, "loess","SmoothingFactor",0.25);
s_yaw = smoothdata(s_yaw, "loess","SmoothingFactor",0.25);
yaw = atan2(s_yaw, c_yaw);

% Computing R and S for each time sample
R = speed./omega_z_f;
S = -v./omega_z_f;
xc = zeros(n,1);
yc = zeros(n,1);

% Defining the threshold to distinguish the curves
radius_threshold = 450;

% Finding fixed centrodes of the curves
for i=1:n
    if sqrt( R(i)^2 + S(i)^2 ) < radius_threshold
        GC_v = [S(i); R(i)];
        GC_f = [-cos( yaw(i) ) sin( yaw(i) ); -sin( yaw(i) ) -cos( yaw(i) )]*GC_v;
        OC_f = GC_f + [xg(i); yg(i)];
        xc(i) = OC_f(1);
        yc(i) = OC_f(2);
    end
end

% Finding starting sample time and ending time of all curves
n_s = 0;                % Vector of the starting times of curves
n_e = 0;                % Vector of the ending times of curves
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
        elseif (xc(i) == 0) && (xc(i-1) ~= 0)
                n_e(k) = i-1;
                k = k+1;
        end
    end
end
% Plotting curves on the path
figure
plot(xg, yg, 'b')
hold on
for i=1:length(n_s)
    plot( xg( n_s(i):n_e(i) ), yg( n_s(i):n_e(i) ), 'r', 'LineWidth', 2 );
end
axis equal
hold off


% Plotting curves and relative fixed centrodes
figure
k = 1;
for j=1:size(n_s,2)
    if n_e(j) - n_s(j) > 50
        subplot(2,7,k)
        plot(xc(n_s(j):n_e(j)),yc(n_s(j):n_e(j)), 'r', 'LineWidth',2)
        hold on
        plot(xg(n_s(j):n_e(j)), yg(n_s(j):n_e(j)), 'b','LineWidth',2)
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

% Plotting curves and relative fixed centrodes on the circuit
figure
plot(xg, yg, 'b')
hold on
k = 1;
for j=1:size(n_s,2)
    if n_e(j) - n_s(j) > 50
        plot(xc(n_s(j):n_e(j)),yc(n_s(j):n_e(j)), 'r', 'LineWidth',2)
        plot(xg(n_s(j):n_e(j)), yg(n_s(j):n_e(j)), 'b','LineWidth',2)
        grid on
        axis equal
       
        k = k+1;
    end
end
hold off

%% Moving centrode
n_c8 = (n_s_c8:1:n_e_c8)';          % Samples of curve 8
n1= n_c8(1);                        % Starting sample
n2 = n_c8(floor(length(n_c8)/2));   % Intermediate sample
n3 = n_c8(end);                     % Final sample
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
plot(xg(n1:n3), yg(n1:n3),'LineWidth',2)
hold on
plot(xc(n1:n3),yc(n1:n3),'LineWidth',2)
for j=1:3
    plot(xm(:,j), ym(:,j), 'LineWidth',1.5)
end
legend("Curve8","Fixed centrode","Moving centrode at t=45.7s","Moving centrode at t=47.33s","Moving centrode at t=48.97s")
axis equal
grid on
hold off

%% Power limited curve 
% Creating the curve samples vector
n_curves =  n_s(1):n_e(1);
for i=2:length(n_s)
    n_curves = [n_curves n_s(i):n_e(i)];
end
n_curves = n_curves';


% Getting instant where pilot is accelerating
farf = smoothdata(farf, "loess","SmoothingFactor",0.25);
n_pl_s = 0;
n_pl_e = 0;
j = 1;
k = 1;
power_threshold = 89;

if farf( n_curves(1) ) >= power_threshold 
    n_pl_s(j) = n_curves(1);
    j = j + 1;
end

for i=2:length(n_curves)
    if ( farf( n_curves(i) ) >= power_threshold ) && ( farf( n_curves(i-1) ) < power_threshold)
        n_pl_s(j) = n_curves(i);
        j = j + 1;
    elseif ( farf( n_curves(i) ) < power_threshold ) && ( farf( n_curves(i-1) ) >= power_threshold)
            n_pl_e(k) = n_curves(i-1);
            k = k + 1;
    end
end

if farf( n_curves(end) ) >= power_threshold
    n_pl_e(k) = n_curves(end);
end

% Plotting power limited curve
figure 
plot(xg, yg, 'b')
hold on
xlabel("[m]")
ylabel("[m]")
grid on
axis equal

for i=1:length(n_pl_s)
    plot( xg( n_pl_s(i):n_pl_e(i) ), yg( n_pl_s(i):n_pl_e(i) ), 'r', 'LineWidth', 2 );
end

%% Grip limited curve
n_gl_s = 0;
n_gl_e = 0;
j = 1;
k = 1;
brake_treshold = 1.5;

if p_brake( n_curves(1) ) >= brake_treshold 
    n_gl_s(j) = n_curves(1);
    j = j + 1;
end

for i=2:length(n_curves)
    if ( p_brake( n_curves(i) ) >= brake_treshold ) && ( p_brake( n_curves(i-1) ) < brake_treshold)
        n_gl_s(j) = n_curves(i);
        j = j + 1;
    elseif ( p_brake( n_curves(i) ) < brake_treshold ) && ( p_brake( n_curves(i-1) ) >= brake_treshold)
            n_gl_e(k) = n_curves(i-1);
            k = k + 1;
    end
end

if p_brake( n_curves(end) ) >= brake_treshold
    n_gl_e(k) = n_curves(end);
end

% Plotting power limited curve
for i=1:length(n_gl_s)
    plot( xg( n_gl_s(i):n_gl_e(i) ), yg( n_gl_s(i):n_gl_e(i) ), 'g', 'LineWidth', 1 );
end
legend([plot( xg( n_pl_s(1):n_pl_e(1) ), yg( n_pl_s(1):n_pl_e(1) ), 'r', 'LineWidth', 2.5 ), ...
    plot( xg( n_gl_s(1):n_gl_e(1) ), yg( n_gl_s(1):n_gl_e(1) ), 'g', 'LineWidth', 2.5 )...
    ], "Power limited", "Grip limited");
hold off


