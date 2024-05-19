% Animations, this script have to be runned after the running of
% vehicle_dynamics_project script

% Animation of moving centrodes of curve 8
figure
for j=1:length(n_c8) 
    for i=1:length(n_c8)
        xm(i,j) = xg(n_c8(j)) - S( i + n1 - 1 )*cos( yaw( n_c8(j) ) ) + R( i + n1 - 1 )*sin( yaw( n_c8(j) ) );
        ym(i,j) = yg(n_c8(j)) - S( i + n1 - 1 )*sin( yaw( n_c8(j) ) ) - R( i + n1 - 1 )*cos( yaw( n_c8(j) ) );
    end
    cla;
    plot(xg(n1:n3), yg(n1:n3));
    hold on;
    grid on;
    plot(xc(n1:n3), yc(n1:n3), 'LineWidth', 2);
    
    % Plotta i nuovi dati per questa iterazione
    plot(xm(:,j), ym(:,j), 'LineWidth', 2);
    
    axis equal
    pause(dt);

end
