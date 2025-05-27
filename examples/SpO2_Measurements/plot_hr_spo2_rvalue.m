% plot_hr_spo2_rvalue.m
% Đọc file dữ liệu từ Serial (đã lưu thành file txt/csv) và vẽ đồ thị HR, SpO2, R-value

% Đọc dữ liệu từ file (giả sử file tên 'data.txt', mỗi dòng: Time, HR, R, SpO2 hoặc HR, R, SpO2)
data = readmatrix('data.txt');

% Kiểm tra số cột
if size(data,2) == 4
    t = data(:,1);
    hr = data(:,2);
    rval = data(:,3);
    spo2 = data(:,4);
elseif size(data,2) == 3
    t = (1:size(data,1))';
    hr = data(:,1);
    rval = data(:,2);
    spo2 = data(:,3);
else
    error('File dữ liệu phải có 3 hoặc 4 cột: [Time,] HR, R-value, SpO2');
end

figure;
subplot(3,1,1);
plot(t, hr, 'r.-');
ylabel('Heart Rate (BPM)');
title('Heart Rate');
grid on;

subplot(3,1,2);
plot(t, spo2, 'b.-');
ylabel('SpO2 (%)');
title('SpO2');
grid on;

subplot(3,1,3);
plot(t, rval, 'k.-');
ylabel('R-value');
xlabel('Time (ms) hoặc Sample');
title('R-value');
grid on;

sgtitle('Heart Rate, SpO2, R-value from MAX3010x');
