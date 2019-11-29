%% Identification
% 1.- Run "systemIdentification" in the command line
% 2.- Choose "time domain"
% 3.- Input "u" Output "y"
% 4.- Select starting time and sample time (in seconds)
% 5.- Select transfer function model

clear all;
close all;
clc;

data = load('samples_1.csv');
u = data(:,1);
y = data(:,2);

%% Validation

clear all;
close all;
clc;

sys = tf([149.1], [1 516.8 375.4]);

% figure(1)
data_50_pwm = load('validation_50_pwm.csv');
data_50_pwm_y = data_50_pwm(:,2);
data_50_t = 0.01:0.01:size(data_50_pwm_y,1)/100;
data_50_time_y = [data_50_t',data_50_pwm_y];
% plot(data_50_t, data_50_pwm_y)
% 
% figure(2)
data_100_pwm = load('validation_100_pwm.csv');
data_100_pwm_y = data_100_pwm(:,2);
data_100_t = 0.01:0.01:size(data_100_pwm_y,1)/100;
data_100_time_y = [data_100_t',data_100_pwm_y];
% plot(data_100_t, data_100_pwm_y)
% 
% figure(3)
data_150_pwm = load('validation_150_pwm.csv');
data_150_pwm_y = data_150_pwm(:,2);
data_150_t = 0.01:0.01:size(data_150_pwm_y,1)/100;
data_150_time_y = [data_150_t',data_150_pwm_y];
% plot(data_150_t, data_150_pwm_y)

% Control Design "M�todo de la Curva de Reacci�n"

sys = tf([149.1], [1 516.8 375.4]);
%sisotool(sys);

Ts = 0.02;  % Tovany notes
R = 0.1073; % Tovany notes
Kc = 1.2/(R*Ts);
Ti = Ts/0.3;
Td = 0.8*Ts;
Kp = Kc
Ki = Kc/Ti
Kd = Kc*Td

%Kp = 100;
%Ki = 8;
%Kd = 100;

% PID of the form : M(n) = M(n-1) + AE(n) + BE(n-1) + CE(n-2)
A = Kp + Ki + Kd
B = -Kp - 2*Kd
C = Kd














%