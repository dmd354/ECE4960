%% Polar---------------------------------------------------------------------
data = table2array(readtable('scan_1_1rot.csv'));
t = (data(:,1)-data(1,1))*10^-6;    %time vector in seconds
theta = -data(:,5)*pi/180;  % angle in rad
gyro = data(:,4);   %gyroscope reading (dps)
rho = data(:,6);
figure(1)
polarplot(theta,rho, '.')
title('Polar Plot')

data2 = table2array(readtable('scan_1_2rot.csv'));
t2 = (data2(:,1)-data(1,1))*10^-6;    %time vector in seconds
theta2 = -data2(:,5)*pi/180;  % angle in rad
gyro2 = data2(:,4);   %gyroscope reading (dps)
rho2 = data2(:,6);
figure(2)
polarplot(theta,rho, '.', theta2,rho2, '.')
title('Polar Plot')
legend('first rotaion', 'second rotation')

%% Tranformation matrices

% scan at (12,10) [in]
xR = 12*25.4;
yR = 10*25.4;
data = table2array(readtable('scan_1_1rot.csv'));

t = (data(:,1)-data(1,1))*10^-6;    %time vector in seconds
theta = -data(:,5)*pi/180;  % angle in rad
gyro = data(:,4);   %gyroscope reading (dps)
distance = data(:,6);
x_1 = zeros(1, length(data));  % vector for x values
y_1 = zeros(1, length(data));  % vector for y values
for k=[1:length(data)]  % for each measurement
    PS = [distance(k); 0; 1];
    robot2world = [cos(theta(k)) -sin(theta(k)) xR;
                   sin(theta(k))  cos(theta(k)) yR;
                   0            0           1 ];
    sensor2robot = [1 0 90
                    0 1 35
                    0 0 1 ];
    PW = robot2world*sensor2robot*PS;
    x_1(k) = PW(1);
    y_1(k) = PW(2);
end


% scan at (44,21) [in]
xR = 44*25.4;   %convert to mm
yR = 19*25.4;   %convert to mm
data = table2array(readtable('x44y21.csv'));

t = (data(:,1)-data(1,1))*10^-6;    %time vector in seconds
theta = -data(:,5)*pi/180;  % angle in rad
gyro = data(:,4);   %gyroscope reading (dps)
distance = data(:,6);
x_2 = zeros(1, length(data));  % vector for x values
y_2 = zeros(1, length(data));  % vector for y values
for k=[1:length(data)]  % for each measurement
    PS = [distance(k); 0; 1];
    robot2world = [cos(theta(k)) -sin(theta(k)) xR;
                   sin(theta(k))  cos(theta(k)) yR;
                   0            0           1 ];
    sensor2robot = [1 0 90
                    0 1 35
                    0 0 1 ];
    PW = robot2world*sensor2robot*PS;
    x_2(k) = PW(1);
    y_2(k) = PW(2);
end

% scan at (10,35) [in]
xR = 10*25.4;   %convert to mm
yR = 33*25.4;   %convert to mm
data = table2array(readtable('x10y35.csv'));

t = (data(:,1)-data(1,1))*10^-6;    %time vector in seconds
theta = -data(:,5)*pi/180;  % angle in rad
gyro = data(:,4);   %gyroscope reading (dps)
distance = data(:,6);
x_3 = zeros(1, length(data));  % vector for x values
y_3 = zeros(1, length(data));  % vector for y values
for k=[1:length(data)]  % for each measurement
    PS = [distance(k); 0; 1];
    robot2world = [cos(theta(k)) -sin(theta(k)) xR;
                   sin(theta(k))  cos(theta(k)) yR;
                   0            0           1 ];
    sensor2robot = [1 0 90
                    0 1 35
                    0 0 1 ];
    PW = robot2world*sensor2robot*PS;
    x_3(k) = PW(1);
    y_3(k) = PW(2);
end

% scan at (37,20) [in]
xR = 37*25.4;   %convert to mm
yR = 18*25.4;   %convert to mm
data = table2array(readtable('x37y20.csv'));

t = (data(:,1)-data(1,1))*10^-6;    %time vector in seconds
theta = -(data(:,5)+360)*pi/180;  % angle in rad
gyro = data(:,4);   %gyroscope reading (dps)
distance = data(:,6);
x_4 = zeros(1, length(data));  % vector for x values
y_4 = zeros(1, length(data));  % vector for y values
for k=[1:length(data)]  % for each measurement
    PS = [distance(k); 0; 1];
    robot2world = [cos(theta(k)) -sin(theta(k)) xR;
                   sin(theta(k))  cos(theta(k)) yR;
                   0            0           1 ];
    sensor2robot = [1 0 90
                    0 1 35
                    0 0 1 ];
    PW = robot2world*sensor2robot*PS;
    x_4(k) = PW(1);
    y_4(k) = PW(2);
end

% scan at (26,15) [in]
xR = 26*25.4;   %convert to mm
yR = 13*25.4;   %convert to mm
data = table2array(readtable('x26y14.csv'));

t = (data(:,1)-data(1,1))*10^-6;    %time vector in seconds
theta = -data(:,5)*pi/180;  % angle in rad
gyro = data(:,4);   %gyroscope reading (dps)
distance = data(:,6);
x_5 = zeros(1, length(data));  % vector for x values
y_5 = zeros(1, length(data));  % vector for y values
for k=[1:length(data)]  % for each measurement
    PS = [distance(k); 0; 1];
    robot2world = [cos(theta(k)) -sin(theta(k)) xR;
                   sin(theta(k))  cos(theta(k)) yR;
                   0            0           1 ];
    sensor2robot = [1 0 90
                    0 1 35
                    0 0 1 ];
    PW = robot2world*sensor2robot*PS;
    x_5(k) = PW(1);
    y_5(k) = PW(2);
end

%plot
figure(3)
plot(x_1,y_1,'.', x_2,y_2,'.', x_3,y_3,'.', x_4,y_4,'.', x_5,y_5,'.')
title('Map')
xlabel('x_{workspace} (mm)'); ylabel('y_{workspace} (mm)');


%% Plot with lines
x_wall = [26 1117 1121 1470 1490 790 759 601 589  75  ];
y_wall = [15 -22  230  225  887  889 676 689 1182 1218];

% close the loop
x_wall = [x_wall x_wall(1)];
y_wall = [y_wall y_wall(1)];

figure(4)
plot(x_1,y_1,'.', x_2,y_2,'.', x_3,y_3,'.', x_4,y_4,'.', x_5,y_5,'.', x_wall,y_wall,'k')
title('Map')
xlabel('x_{workspace} (mm)'); ylabel('y_{workspace} (mm)');

