%% Ramp---------------------------------------------------------------------
data = table2array(readtable('Ramp_data.csv'));
t = (data(:,1)-data(1,1))*10^-6;    %time vector in seconds
u = data(:,2);  % control input
gyro = data(:,4);   %gyroscope reading (dps)
figure(1)
plot(t,u, t,gyro)
title('Ramp Response')
xlabel('time(s)')
legend('control input', 'gyro (dps)')

%% Step---------------------------------------------------------------------
figure(2); tiledlayout(3,2)
% u=200
data = table2array(readtable('OL_input_200.csv'));
t = (data(:,1)-data(1,1))*10^-6;    %time vector in seconds
u = data(:,2);  % control input
gyro = data(:,4);   %gyroscope reading (dps)
nexttile
plot(t,u, t,gyro)
title('Step Response, input=200u(t)')
xlabel('time(s)')
l=legend('control input', 'gyro (dps)'); l.Location = 'southeast';

% u=180
data = table2array(readtable('OL_input_180.csv'));
t = (data(:,1)-data(1,1))*10^-6;    %time vector in seconds
u = data(:,2);  % control input
gyro = data(:,4);   %gyroscope reading (dps)
nexttile
plot(t,u, t,gyro)
title('Step Response, input=180')
xlabel('time(s)')
l=legend('control input', 'gyro (dps)'); l.Location = 'southeast';

% u=170
data = table2array(readtable('OL_input_170.csv'));
t = (data(:,1)-data(1,1))*10^-6;    %time vector in seconds
u = data(:,2);  % control input
gyro = data(:,4);   %gyroscope reading (dps)
nexttile
plot(t,u, t,gyro)
title('Step Response, input=170')
xlabel('time(s)')
l=legend('control input', 'gyro (dps)'); l.Location = 'southeast';

% u=160
data = table2array(readtable('OL_input_160.csv'));
t = (data(:,1)-data(1,1))*10^-6;    %time vector in seconds
u = data(:,2);  % control input
gyro = data(:,4);   %gyroscope reading (dps)
nexttile
plot(t,u, t,gyro)
title('Step Response, input=160')
xlabel('time(s)')
l=legend('control input', 'gyro (dps)'); l.Location = 'southeast';

% u=150
data = table2array(readtable('OL_input_150.csv'));
t = (data(:,1)-data(1,1))*10^-6;    %time vector in seconds
u = data(:,2);  % control input
gyro = data(:,4);   %gyroscope reading (dps)
nexttile
plot(t,u, t,gyro)
title('Step Response, input=150')
xlabel('time(s)')
l=legend('control input', 'gyro (dps)');
l.Location = 'southeast';

% u=140
data = table2array(readtable('OL_input_140.csv'));
t = (data(:,1)-data(1,1))*10^-6;    %time vector in seconds
u = data(:,2);  % control input
gyro = data(:,4);   %gyroscope reading (dps)
nexttile
plot(t,u, t,gyro)
title('Step Response, input=150')
xlabel('time(s)')
l=legend('control input', 'gyro (dps)'); l.Location = 'southeast';

%% Feedback-----------------------------------------------------------------
figure(3); tiledlayout(2,2);
setpoint = 900;

data = table2array(readtable('kp1.csv'));
t = (data(:,1)-data(1,1))*10^-6;    %time vector in seconds
u = data(:,2);  % control input
gyro = data(:,4);   %gyroscope reading (dps)
nexttile
plot(t,setpoint*ones(length(t)), t,u, t,gyro)
title('K_p=1')
xlabel('time(s)')
l=legend('setpoint', 'control input', 'gyro (dps)'); l.Location = 'southeast';