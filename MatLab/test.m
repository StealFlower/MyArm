clc;close all;clear all;
% 设置参数
fs = 500;          % 采样频率 (Hz)
f_cutoff = 2;    % 截止频率 (Hz)
omega_c = 2 * pi * f_cutoff; % 角频率
epsilon = 0.5;   % 阻尼比
Tsw = 1 / fs;    % 采样周期

% 生成测试信号
t = 0:Tsw:10;                     % 时间向量 0到10秒
x = sin(2*pi*1*t) + 0.5 * randn(size(t)); % 1Hz的正弦波加上随机噪声

% 初始化输出信号
y = zeros(size(x));

% 应用二阶低通滤波器
for i = 1:length(x)
    y(i) = fcn_filter_Order2(x(i), omega_c, epsilon, Tsw);
end

% 绘制结果
figure;
subplot(2, 1, 1);
plot(t, x, 'r'); % 原始信号
title('原始信号');
xlabel('时间 (秒)');
ylabel('幅值');

subplot(2, 1, 2);
plot(t, y, 'b'); % 滤波后的信号
title('滤波后的信号');
xlabel('时间 (秒)');
ylabel('幅值');

function y = fcn_filter_Order2(x, omega_c, epsilon, Tsw)
%二阶低通滤波器，采样频率50Hz，截止频率暂定f=50Hz
%ω=2*pi*f
 
b0 = omega_c*omega_c * Tsw*Tsw;
a0 = b0 + 4*epsilon*omega_c*Tsw + 4;
a1 = 2*b0 - 8;
a2 = b0 - 4*epsilon*omega_c*Tsw + 4;
 
persistent p_x;
if isempty(p_x)
    p_x = zeros(2,1);
end
 
persistent p_y;
if isempty(p_y)
    p_y = zeros(2,1);
end
 
y = (b0*x + 2*b0*p_x(1) + b0*p_x(2) - a1*p_y(1) - a2*p_y(2)) / a0;
 
p_x(2) = p_x(1);
p_x(1) = x;
 
p_y(2) = p_y(1);
p_y(1) = y;
end