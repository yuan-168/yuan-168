% 文件名列表
filenames = {'/Users/shiyuanwang/Documents/001 individual project/project/30/30 pressure tracking/120v60', ...
              '/Users/shiyuanwang/Documents/001 individual project/project/30/30 pressure tracking/130V60', ...
              '/Users/shiyuanwang/Documents/001 individual project/project/30/30 pressure tracking/140V60', ...
              '/Users/shiyuanwang/Documents/001 individual project/project/30/30 pressure tracking/150V60', ...
              '/Users/shiyuanwang/Documents/001 individual project/project/30/30 pressure tracking/160V60'};
% 初始化变量

all_pressures = {'pressures_120', 'pressures_130', 'pressures_140', 'pressures_150', 'pressures_160'};

% 读取每个文件并提取压力数据
for i = 1:length(filenames)
    % 打开当前文件
    filename = filenames{i};
    fileID = fopen(filename, 'r');
    
    % 初始化当前文件的压力数组
    pressures = [];
    
    % 逐行读取数据
    line = fgetl(fileID);
    while ischar(line)
        % 检查行是否包含 "Pressure (KPa):"
        if contains(line, 'Pressure (KPa):')
            % 提取压力值
            splitLine = strsplit(line, 'Pressure (KPa):');
            pressureStr = strtrim(splitLine{2}); % 去掉前后的空格
            pressureValue = str2double(pressureStr); % 将字符串转换为数值
            
            % 将压力值添加到数组
            pressures = [pressures; pressureValue]; %#ok<AGROW>
        end
        % 读取下一行
        line = fgetl(fileID);
    end
    
    % 关闭文件
    fclose(fileID);
    
    % 将当前文件的压力数据保存到 cell 数组中
    all_pressures{i} = pressures;
end

% 使用 violin.m 绘制小提琴图
figure;

violin(all_pressures, 'GroupLabels', arrayfun(@num2str, 1:length(filenames), 'UniformOutput', false), ...
       'ShowMean', true, 'ShowMedian', true);

% 设置图形属性
xlabel(' ');
ylabel('Pressure Value (KPa) ');
title('Pressure Distribution (Violin Plot)/Velocity 60/Thickness 30');
% 设置x轴刻度为1, 2, 3, 4, 5
xticks(1:5);
xticklabels(arrayfun(@num2str, 1:5, 'UniformOutput', false));

% 设置y轴范围为100-200 KPa
ylim([100 180]);
grid on;
savefig('/Users/shiyuanwang/Documents/individual/data and staticstics/30/pressure plot/vel60');
