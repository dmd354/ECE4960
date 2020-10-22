dt = 0.045;  % time between sensor readings (s)
omega = 30; % turning speed (deg/sec)
r = 2;   % distance from wall (m)
start_angle = 45;    % start angle (deg)
final_angle = start_angle+omega*dt;
d0 = r/cos(start_angle*pi/180);
d1 = r/cos(final_angle*pi/180);
w0 = d0*sin(start_angle*pi/180);
w1 = d1*sin(final_angle*pi/180);

fprintf('orientation changed by %d degrees\n', final_angle-start_angle);
fprintf('distance reading changed by %d mm\n', (d1-d0)*1000);
fprintf('distance along wall between readings = %d mm\n\n', (w1-w0)*1000);