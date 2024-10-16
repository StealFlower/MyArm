% 清理工作空间  
clear; clc; close all;  

% 参数定义  
K1 = 50;  % 增益系数  
K2 = 0.5; % 另一增益系数  
dt = 0.01; % 时间步长  
t_end = 10; % 模拟时间  
t0 = 1; % 观测器启动时间  
threshold = 5; % 碰撞检测阈值
% 初始化变量  
time = 0:dt:t_end; % 时间向量  
r = zeros(size(time)); % 观测器输出  
e = zeros(size(time)); % 观测器输入，假设某个活动信号  
collision_force = zeros(size(time)); % 碰撞力  

% 模拟输入信号（假设为某个动量信号的阶跃信号）  
for i = 1:length(time)  
    if time(i) > 2  
        e(i) = 10; % 假设在t>2时有一个阶跃信号  
    else  
        e(i) = 0;  
    end  
end  

% 观测器动态更新  
for i = 2:length(time)  
    % 计算输入的动量差  
    momentum_diff = K2 * r(i-1) - K1 * e(i-1);  

    % 更新观测器输出  
    r(i) = r(i-1) + dt * momentum_diff;  

    % 碰撞力（可以定义为观测器的输出）  
    collision_force(i) = r(i);  

    % 判断是否发生碰撞  
    if abs(collision_force(i)) > threshold % 设定碰撞检测阈值  
        fprintf('Collision detected at t = %.2f seconds\n', time(i));  
    end  
end  

% 绘制结果  
figure;  
subplot(3,1,1);  
plot(time, e, 'r', 'LineWidth', 1.5);  
title('Input Signal (Momentum Difference)');  
xlabel('Time (s)');  
ylabel('e(t)');  
grid on;  

subplot(3,1,2);  
plot(time, r, 'b', 'LineWidth', 1.5);  
title('Observer Output r(t)');  
xlabel('Time (s)');  
ylabel('r(t)');  
grid on;  

subplot(3,1,3);  
plot(time, collision_force, 'g', 'LineWidth', 1.5);  
title('Collision Force Estimate');  
xlabel('Time (s)');  
ylabel('Force (N)');  
grid on;  
  
