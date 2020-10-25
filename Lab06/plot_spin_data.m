%%Polar---------------------------------------------------------------------
data = table2array(readtable('scan_1.csv'));
t = (data(:,1)-data(1,1))*10^-6;    %time vector in seconds
u = data(:,2);  % control input
gyro = data(:,4);   %gyroscope reading (dps)
figure(1)
plot(t,u, t,gyro)
title('Ramp Response')
xlabel('time(s)')
legend('control input', 'gyro (dps)')


