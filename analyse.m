% 读取数据
filename = '/Users/shiyuanwang/Documents/001 individual project/project/50/Tip Tracking/140KPa_V20.txt';
fileID = fopen(filename, 'r');
data = textscan(fileID, '%s %f %f %f', 'Delimiter', ' ', 'MultipleDelimsAsOne', 1);
fclose(fileID);

% 提取时间和坐标数据
time_str = data{1};
x = data{2};
y = data{3};
z = data{4};

for i = 2:length(x)
    if x(i) == 0 && y(i) == 0 && z(i) == 0
        % 用前一个有效的位置数据替换
        x(i) = x(i-1);
        y(i) = y(i-1);
        z(i) = z(i-1);
    end
end

% 计算到初始点的距离
initial_point = [x(1), y(1), z(1)];
distance_from_initial = sqrt((x - initial_point(1)).^2 + (y - initial_point(2)).^2 + (z - initial_point(3)).^2);

% 将时间字符串转换为秒数
time = datetime(time_str, 'InputFormat', 'HH:mm:ss.SSS');
time_seconds = seconds(time - time(1));

% 设置延拓长度
extend_len = 10;

% 取前十个数
first_five = distance_from_initial(1:extend_len);

% 取后十个数
last_five = distance_from_initial(end-extend_len+1:end);


% 重复前个数并拼接到开头
padded_data = [first_five; distance_from_initial];

% 重复后个数并拼接到结尾
padded_data = [padded_data; last_five];

% 应用低通滤波器平滑距离数据
cutoff_frequency = 0.01; 
smoothed_distance = lowpass(padded_data, cutoff_frequency, 1/(time_seconds(2) - time_seconds(1)));

% 去除延拓部分，得到滤波后的数据
smoothed_distance = smoothed_distance(extend_len+1:end - extend_len);

disp(length(time_seconds));
disp(length(smoothed_distance));
% 使平滑后的距离从0开始
smoothed_distance = smoothed_distance - smoothed_distance(1);

% 计算速度（简单的有限差分法），使用平滑后的距离数据
num_points = length(smoothed_distance);  % 获取数据点的数量
original_speed = zeros(num_points - 1, 1);  % 初始化速度数组
speed_time = zeros(num_points - 1, 1);  % 初始化时间数组

for i = 1:(num_points - 1)
    dt = time_seconds(i+1) - time_seconds(i);  % 计算两个时间点之间的时间间隔
    dd = smoothed_distance(i+1) - smoothed_distance(i);  % 计算两个位置点之间的距离差
    original_speed(i) = dd / dt;  % 计算速度并存储
    
    % 在计算速度的同时，插入时间点
    speed_time(i) = time_seconds(i) + dt / 2;  % 插入两个时间点之间的中点时间
end

% 再次应用低通滤波器平滑速度数据
cutoff_frequency1 = 0.0005; 
smoothed_speed = lowpass(original_speed, cutoff_frequency1, 1/(time_seconds(2) - time_seconds(1)));

% 绘制初始点到每个点的距离和速度-时间曲线
figure;
yyaxis left
plot(time_seconds, distance_from_initial, '-o', 'DisplayName', 'Original Position', 'Color', [0.5, 0.5, 0.5]); % 原始距离曲线，灰色
hold on;
plot(time_seconds, smoothed_distance, '-b', 'LineWidth', 2, 'DisplayName', 'Distance'); % 平滑后的距离曲线，蓝色加粗
ylabel('Distance from Initial Point (units)');
ylim([-10 120]); % 确保y轴从0开始

yyaxis right
plot(speed_time, original_speed, '-r', 'DisplayName', 'Speed'); % 原始速度曲线，红色
hold on;
plot(speed_time, smoothed_speed, '-m', 'LineWidth', 2, 'DisplayName', 'filtered Speed'); % 平滑后的速度曲线，草绿色加粗
xlabel('Time (seconds)');
ylabel('Speed (units/second)');
xlim([0 70]);
ylim([-1 10]); % 确保y轴从0开始

title('Pressure160KPa - Velocity20 - Thickness50um');
legend show;
grid on;