data = table2array(readtable('OL_spin_data2.csv'));
t = (data(:,1)-data(1,1))*10^-6;    %time vector in seconds
u = data(:,2);  % control input
gyro = data(:,4);   %gyroscope reading
yaw = data(:,5);
figure(1)
plot(t,u, t,gyro)
title('Ramp Response')
xlabel('time(s)')
legend('control input', 'gyro output','yaw')