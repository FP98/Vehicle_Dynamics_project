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
    
    plot(xm(:,j), ym(:,j), 'LineWidth', 2);
    plot(xg(j + n1 -1), yg(j + n1 -1), 'o', 'MarkerSize', 10, 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');
    
    axis equal
    pause(0.00001);
end
hold off
