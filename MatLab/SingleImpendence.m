     clc
clear
close all

% 阻抗控制仿真

% 系统参数
m = 1;      % 质量 (kg)
k = 100;     % 弹簧常数 (N/m)
b = 18;      % 阻尼系数 (N·s/m)

% 控制目标
desired_position = 1; % 期望位置 (m)
desired_velocity = 0;  % 期望速度 (m/s)

% 仿真参数
dt = 0.01;           % 时间步长 (s)
t_total = 10;        % 总仿真时间 (s)
time = 0:dt:t_total; % 时间向量

% 初始化变量
position = zeros(size(time)); % 当前位置 (m)
velocity = zeros(size(time)); % 当前速度 (m/s)
torque = zeros(size(time));
% 主循环
for i = 2:length(time)
    if i == 20
        desired_position = 3;
    else
        desired_position = 1;
    end    % 计算当前位置的误差
    position_error = desired_position - position(i-1);
    velocity_error = desired_velocity - velocity(i-1);
    
    % 阻抗控制力
    force = k * position_error + b * velocity_error; 
    
    % 更新加速度
    acceleration = force / m; 
    
    % 更新速度和位置
    velocity(i) = velocity(i-1) + acceleration * dt;
    position(i) = position(i-1) + velocity(i) * dt;
    torque(i) = force;
end

% 绘制结果
figure;
subplot(3,1,1);
plot(time, position, 'b', 'LineWidth', 2);
hold on;
yline(desired_position, 'r--', '期望位置');
xlabel('时间 (s)');
ylabel('位置 (m)');
title('位置随时间变化');
grid on;

subplot(3,1,2);
plot(time, velocity, 'g', 'LineWidth', 2);
xlabel('时间 (s)');
ylabel('速度 (m/s)');
title('速度随时间变化');
grid on;

subplot(3,1,3);
plot(time, torque, 'r', 'LineWidth', 2);
xlabel('时间 (s)');
ylabel('力 (N)');
title('力随时间变化');
grid on;