% 文件路径和对应的标签
file_paths = { ...
    '/Users/shiyuanwang/Documents/001 individual project/40/40um/120KPa_V40.txt', ...
    '/Users/shiyuanwang/Documents/001 individual project/40/40um/130KPa_V40.txt', ...
    '/Users/shiyuanwang/Documents/001 individual project/40/40um/140KPa_V40.txt', ...
    '/Users/shiyuanwang/Documents/001 individual project/40/40um/150KPa_V40.txt', ...
    '/Users/shiyuanwang/Documents/001 individual project/40/40um/160KPa_V40.txt' ...
};

% 创建一个包含 5 个数据集的图形窗口
figure;
hold on;  % 保持图形窗口，以便绘制多个数据集

% 用于存储不同的纵坐标轴范围
y_limits = zeros(length(file_paths), 2);

% 定义颜色数组 (RGB)
colors = [
    [237/255, 173/255, 197/255];
    [206/255, 170/255, 208/255];
    [149/255, 132/255, 193/255];
    [108/255, 190/255, 195/255];
    [97/255, 156/255, 217/255];
];
% 遍历每个数据文件
for file_index = 1:length(file_paths)
    filename = file_paths{file_index};
    
    % 从文件路径中提取标签
    [~, file_name, ~] = fileparts(filename);  % 获取文件名（不带扩展名）
    
    % 读取数据文件
    fileID = fopen(filename, 'r');
    data = textscan(fileID, '%s %f %f %f', 'Delimiter', ' ', 'MultipleDelimsAsOne', 1);
    fclose(fileID);

    % 提取时间和坐标数据
    time_str = data{1};
    x = data{2}; % 直接提取
    y = data{3}; % 直接提取
    z = data{4}; % 直接提取

    % 替换坐标为零的点
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
    first_five = repmat(distance_from_initial(1), extend_len, 1);

    % 取后十个数
    last_five = repmat(distance_from_initial(end), extend_len, 1);

    % 重复前个数并拼接到开头
    padded_data = [first_five; distance_from_initial];

    % 重复后个数并拼接到结尾
    padded_data = [padded_data; last_five];

    % 应用低通滤波器平滑距离数据
    cutoff_frequency = 0.01; 
    smoothed_distance = lowpass(padded_data, cutoff_frequency, 1/dt);

    % 去除延拓部分，得到滤波后的数据
    smoothed_distance = smoothed_distance(extend_len+1:end - extend_len);
    
    disp(length(time_seconds));
    disp(length(smoothed_distance));
    
    % 使平滑后的距离从0开始
    smoothed_distance = smoothed_distance - smoothed_distance(1);
    
    % 确保时间范围在 [end-60, end] 内
    time_end = time_seconds(end);
    time_start = time_end - 60;
    
    % 找到对应 [end-60, end] 的索引范围
    indices = time_seconds >= time_start & time_seconds <= time_end;

    % 缩短时间和距离数据
    shortened_time_seconds = time_seconds(indices);
    shortened_smoothed_distance = smoothed_distance(indices);
    disp(length(shortened_time_seconds));
    disp(length(shortened_smoothed_distance));
    % 将 `time_start-1` 设置为新的起点
    shortened_time_seconds = shortened_time_seconds - shortened_time_seconds(1);

    % 绘制数据（使用不同的颜色）
    %plot(shortened_time_seconds, shortened_smoothed_distance, 'LineWidth', 2, 'DisplayName', file_name);
    % 绘制数据（使用指定颜色）
    plot(shortened_time_seconds, shortened_smoothed_distance, 'LineWidth', 3, ...
         'Color', colors(file_index, :), 'DisplayName', file_name);
end

% 添加图例和标签
xlabel('Time (seconds)');
ylabel('Grow Length');
title('Grow Length for different pressure - 40um');
legend('show', 'FontWeight', 'bold'); % 显示图例并设置字体加粗

grid on;  % 添加网格
hold off;  % 释放图形窗口