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
xlim([0,5]);

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
xlim([0,5]);

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
xlim([0,5]);

% u=160
data = table2array(readtable('OL_input_160.csv'));
t = (data(:,1)-data(1,1))*10^-6;    %time vector in seconds
u = data(:,2);  % control input
gyro = data(:,4);   %gyroscope reading (dps)
nexttile
plot(t,u, t,gyro)
title('Step Response, input=160')
xlabel('time(s)')
l=legend('control input', 'gyro (dps)'); 
l.Location = 'southeast';
xlim([0,5]);

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
xlim([0,5]);

% u=140
data = table2array(readtable('OL_input_140.csv'));
t = (data(:,1)-data(1,1))*10^-6;    %time vector in seconds
u = data(:,2);  % control input
gyro = data(:,4);   %gyroscope reading (dps)
nexttile
plot(t,u, t,gyro)
title('Step Response, input=150')
xlabel('time(s)')
l=legend('control input', 'gyro (dps)'); 
l.Location = 'southeast';
xlim([0,5]);

%% P Control -----------------------------------------------------------------
figure(3); tiledlayout(3,1);


% Kp=1
data = table2array(readtable('kp1.csv'));
t = (data(:,1)-data(1,1))*10^-6;    %time vector in seconds
u = data(:,2);  % control input
gyro = data(:,4);   %gyroscope reading (dps)
setpoint = 900*ones(length(t));
nexttile
plot(t,u,'b', t,gyro,'r', t,setpoint,'g')
title('K_p=1')
xlabel('time(s)')
xlim([0,2])

% Kp=2
data = table2array(readtable('kp2.csv'));
t = (data(:,1)-data(1,1))*10^-6;    %time vector in seconds
u = data(:,2);  % control input
gyro = data(:,4);   %gyroscope reading (dps)
setpoint = 900*ones(length(t));
nexttile
plot(t,u,'b', t,gyro,'r', t,setpoint,'g')
title('K_p=2')
xlabel('time(s)')
xlim([0,2])

% Kp=5
data = table2array(readtable('kp5.csv'));
t = (data(:,1)-data(1,1))*10^-6;    %time vector in seconds
u = data(:,2);  % control input
gyro = data(:,4);   %gyroscope reading (dps)
setpoint = 900*ones(length(t));
nexttile
plot(t,u,'b', t,gyro,'r', t,setpoint,'g')
title('K_p=5')
xlabel('time(s)')
xlim([0,2])

%% PI control

figure(3); tiledlayout(2,2);

% Kp=1, Ki=0.1
data = table2array(readtable('kp1_ki0p1.csv'));
t = (data(:,1)-data(1,1))*10^-6;    %time vector in seconds
u = data(:,2);  % control input
gyro = data(:,4);   %gyroscope reading (dps)
setpoint = 900*ones(length(t));
nexttile
plot(t,u,'b', t,gyro,'r', t,setpoint,'g')
title('K_p=1, K_i=0.1')
xlabel('time(s)')
xlim([0,5])

% Kp=1.5, Ki=0.5
data = table2array(readtable('kp1p5_ki0p5.csv'));
t = (data(:,1)-data(1,1))*10^-6;    %time vector in seconds
u = data(:,2);  % control input
gyro = data(:,4);   %gyroscope reading (dps)
setpoint = 900*ones(length(t));
nexttile
plot(t,u,'b', t,gyro,'r', t,setpoint,'g')
title('K_p=1.5, K_i=0.5')
xlabel('time(s)')
xlim([0,5])

% Kp=1.8, Ki=0.8
data = table2array(readtable('kp1p8_ki0p8.csv'));
t = (data(:,1)-data(1,1))*10^-6;    %time vector in seconds
u = data(:,2);  % control input
gyro = data(:,4);   %gyroscope reading (dps)
setpoint = 900*ones(length(t));
nexttile
plot(t,u,'b', t,gyro,'r', t,setpoint,'g')
title('K_p=1.8, K_i=0.8')
xlabel('time(s)')
xlim([0,5])

% Kp=2, Ki=1
data = table2array(readtable('kp2_ki1.csv'));
t = (data(:,1)-data(1,1))*10^-6;    %time vector in seconds
u = data(:,2);  % control input
gyro = data(:,4);   %gyroscope reading (dps)
setpoint = 900*ones(length(t));
nexttile
plot(t,u,'b', t,gyro,'r', t,setpoint,'g')
title('K_p=2, K_i=1')
xlabel('time(s)')
xlim([0,5])

%% Setpoint----------------------------------------------------------------

figure(4); tiledlayout(2,2);

% 500
data = table2array(readtable('sp500.csv'));
t = (data(:,1)-data(1,1))*10^-6;    %time vector in seconds
u = data(:,2);  % control input
gyro = data(:,4);   %gyroscope reading (dps)
setpoint = 500*ones(length(t));
nexttile
plot(t,gyro,'r', t,setpoint,'g')
title('Setpoint=500')
xlabel('time(s)')
xlim([0,5])

% 300
data = table2array(readtable('sp300.csv'));
t = (data(:,1)-data(1,1))*10^-6;    %time vector in seconds
u = data(:,2);  % control input
gyro = data(:,4);   %gyroscope reading (dps)
setpoint = 300*ones(length(t));
nexttile
plot(t,gyro,'r', t,setpoint,'g')
title('Setpoint=300')
xlabel('time(s)')
xlim([0,5])

% 200
data = table2array(readtable('sp200.csv'));
t = (data(:,1)-data(1,1))*10^-6;    %time vector in seconds
u = data(:,2);  % control input
gyro = data(:,4);   %gyroscope reading (dps)
setpoint = 200*ones(length(t));
nexttile
plot(t,gyro,'r', t,setpoint,'g')
title('Setpoint=200')
xlabel('time(s)')
xlim([0,5])

% 100
data = table2array(readtable('sp100.csv'));
t = (data(:,1)-data(1,1))*10^-6;    %time vector in seconds
u = data(:,2);  % control input
gyro = data(:,4);   %gyroscope reading (dps)
setpoint = 100*ones(length(t));
nexttile
plot(t,gyro,'r', t,setpoint,'g')
title('Setpoint=100')
xlabel('time(s)')
xlim([0,5])

%% one side-----------------------------------------------------------------

figure(5); tiledlayout(2,2);

% 50
data = table2array(readtable('one_side_sp50.csv'));
t = (data(:,1)-data(1,1))*10^-6;    %time vector in seconds
u = data(:,2);  % control input
gyro = data(:,4);   %gyroscope reading (dps)
setpoint = 50*ones(length(t));
nexttile
plot(t,gyro,'r', t,setpoint,'g')
title('Setpoint=50')
xlabel('time(s)')
xlim([0,12])

% 40
data = table2array(readtable('one_side_sp40.csv'));
t = (data(:,1)-data(1,1))*10^-6;    %time vector in seconds
u = data(:,2);  % control input
gyro = data(:,4);   %gyroscope reading (dps)
setpoint = 40*ones(length(t));
nexttile
plot(t,gyro,'r', t,setpoint,'g')
title('Setpoint=40')
xlabel('time(s)')
xlim([0,12])

% 30
data = table2array(readtable('one_side_sp30.csv'));
t = (data(:,1)-data(1,1))*10^-6;    %time vector in seconds
u = data(:,2);  % control input
gyro = data(:,4);   %gyroscope reading (dps)
setpoint = 30*ones(length(t));
nexttile
plot(t,gyro,'r', t,setpoint,'g')
title('Setpoint=30')
xlabel('time(s)')
xlim([0,12])

% 20
data = table2array(readtable('one_side_sp30.csv'));
t = (data(:,1)-data(1,1))*10^-6;    %time vector in seconds
u = data(:,2);  % control input
gyro = data(:,4);   %gyroscope reading (dps)
setpoint = 20*ones(length(t));
nexttile
plot(t,gyro,'r', t,setpoint,'g')
title('Setpoint=20')
xlabel('time(s)')
xlim([0,12])
