%% scan at (12,12) [in]
xR = 12*25.4;
yR = 12*25.4;

theta = (0:20:340)*pi/180;  % angle in rad
% selected distances from python script
distance = [1067. 1072. 1074. 1089. 1146. 1248.  571.  701.  860.  832.  228.  175.  188.  301.  268.  245.  310.  516.];
x_1 = zeros(1, 18);  % vector for x values
y_1 = zeros(1, 18);  % vector for y values
for k=[1:18]  % for each measurement
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


x_wall = [0 1117 1121 1470 1490 790 759 601 589  75  ];
y_wall = [0 -22  230  225  887  889 676 689 1182 1218];

% close the loop
x_wall = [x_wall x_wall(1)];
y_wall = [y_wall y_wall(1)];

figure(1); tiledlayout(2,2);
nexttile;
plot(xR,yR,'*', x_1,y_1,'.', x_wall,y_wall,'k')
title('Measurements from (12",12")')
xlabel('x_{workspace} (mm)'); ylabel('y_{workspace} (mm)');
legend('robot', 'measurements', 'walls')


%% scan at (24,11) [in]
xR = 24*25.4;
yR = 11*25.4;

theta = (0:20:340)*pi/180;  % angle in rad
% selected distances from python script
distance = [768. 771. 768. 772. 834. 868. 962. 860. 639. 393. 795. 986. 608. 522. 452. 258. 266. 453.];
x_1 = zeros(1, 18);  % vector for x values
y_1 = zeros(1, 18);  % vector for y values
for k=[1:18]  % for each measurement
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


nexttile;
plot(xR,yR,'*', x_1,y_1,'.', x_wall,y_wall,'k')
title('Measurements from (24",11")')
xlabel('x_{workspace} (mm)'); ylabel('y_{workspace} (mm)');
legend('robot', 'measurements', 'walls')



%% scan at (42,23) [in]
xR = 42*25.4;
yR = 23*25.4;

theta = (0:20:340)*pi/180;  % angle in rad
% selected distances from python script
distance = [ 326.  328.  346.  411.  287.  226.  210.  245.  373.  322. 1035. 1031. 650.  588.  408.  424.  336.  380.];
x_1 = zeros(1, 18);  % vector for x values
y_1 = zeros(1, 18);  % vector for y values
for k=[1:18]  % for each measurement
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


nexttile;
plot(xR,yR,'*', x_1,y_1,'.', x_wall,y_wall,'k')
title('Measurements from (42",23")')
xlabel('x_{workspace} (mm)'); ylabel('y_{workspace} (mm)');
legend('robot', 'measurements', 'walls')

%% scan at (12,33) [in]
xR = 12*25.4;
yR = 33*25.4;

theta = (0:20:340)*pi/180;  % angle in rad
% selected distances from python script
distance = [ 326.  328.  346.  411.  287.  226.  210.  245.  373.  322. 1035. 1031.  650.  588.  408.  424.  336.  380.];
x_1 = zeros(1, 18);  % vector for x values
y_1 = zeros(1, 18);  % vector for y values
for k=[1:18]  % for each measurement
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


nexttile;
plot(xR,yR,'*', x_1,y_1,'.', x_wall,y_wall,'k')
title('Measurements from (12",33")')
xlabel('x_{workspace} (mm)'); ylabel('y_{workspace} (mm)');
legend('robot', 'measurements', 'walls')