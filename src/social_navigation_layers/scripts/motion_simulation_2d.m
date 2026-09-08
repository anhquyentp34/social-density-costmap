% motion_simulation_2d.m
% Mô phỏng mô hình proxemic có tính đến vận tốc góc của người đi bộ
% Tích hợp social cost visualization

clear; clc; close all;

% Thêm đường dẫn thư viện librobotics
addpath(genpath('./librobotics'));

fprintf('🎯 MÔ PHỎNG PROXEMIC - VẬN TỐC GÓC NGƯỜI ĐI BỘ\n');
fprintf('==================================================\n');

%% 1. THAM SỐ MÔ PHỎNG
v = 2.0;           % Vận tốc tịnh tiến (m/s)
w = 0.0;           % Vận tốc góc quay (rad/s) - quan trọng cho proxemic
sim_time = 10;     % Thời gian mô phỏng (s)
dt = 0.05;         % Bước thời gian (s)

% Vị trí ban đầu
x0 = 0; y0 = 0; theta0 = 0;

% Tham số Social Cost (Proxemic)
amplitude = 77.0;      % Biên độ cực đại
sigma_base = 0.25;    % Phương sai cơ bản
cutoff = 10.0;         % Ngưỡng chi phí tối thiểu
factor_param = 2.0;  % Hệ số vận tốc

fprintf('📊 Tham số: v=%.1f m/s, w=%.1f rad/s, t=%.1f s\n', v, w, sim_time);
fprintf('📊 Tham số Proxemic: amplitude=%d, sigma=%.1f, cutoff=%d\n', amplitude, sigma_base, cutoff);

%% 2. TÍNH TOÁN QUỸ ĐẠO NGƯỜI ĐI BỘ
t = 0:dt:sim_time;
N = length(t);
x = zeros(1, N);
y = zeros(1, N);
theta = zeros(1, N);
velocity_x = zeros(1, N);
velocity_y = zeros(1, N);

x(1) = x0; y(1) = y0; theta(1) = theta0;

% Tính vận tốc ban đầu
velocity_x(1) = v * cos(theta(1));
velocity_y(1) = v * sin(theta(1));

for i = 2:N
    theta(i) = theta(i-1) + w * dt;
    vx = v * cos(theta(i));
    vy = v * sin(theta(i));
    x(i) = x(i-1) + vx * dt;
    y(i) = y(i-1) + vy * dt;
    velocity_x(i) = vx;
    velocity_y(i) = vy;
end

%% 3. TẠO LƯỚI CHO PROXEMIC COST
% Tạo lưới xung quanh quỹ đạo
x_range = [min(x)-3, max(x)+3];
y_range = [min(y)-3, max(y)+3];

[X_grid, Y_grid] = meshgrid(linspace(x_range(1), x_range(2), 120), ...
                           linspace(y_range(1), y_range(2), 120));

%% 4. TÍNH TOÁN PROXEMIC COST
fprintf('\n🔄 Đang tính toán proxemic cost...\n');
proxemic_cost_total = zeros(size(X_grid));

% Tính proxemic cost cho các điểm dự đoán
num_prediction_points = min(40, N);  % Đảm bảo không vượt quá số điểm có sẵn
fprintf('   Số điểm dự đoán: %d\n', num_prediction_points);
fprintf('   Khoảng thời gian giữa các điểm: %.3f s\n', dt);

% Bắt đầu từ vị trí t=0 (thời điểm ban đầu)
for k = 1:num_prediction_points
    person_pos = [x(k), y(k)];
    velocity = [velocity_x(k), velocity_y(k)];
    
    % Tính vận tốc và factor
    velocity_mag = sqrt(velocity(1)^2 + velocity(2)^2);
    velocity_angle = atan2(velocity(2), velocity(1));
    factor = 1.0 + velocity_mag * factor_param;
    
    % Tính proxemic cost cho từng điểm trong lưới
    for i = 1:size(X_grid, 1)
        for j = 1:size(X_grid, 2)
            point = [X_grid(i, j), Y_grid(i, j)];
            
            % Tính khoảng cách và góc từ người đến điểm
            person_to_point_vector = point - person_pos;
            person_to_point_angle = atan2(person_to_point_vector(2), person_to_point_vector(1));
            
            % Tính góc chênh lệch so với hướng vận tốc
            angle_diff = wrapToPi(person_to_point_angle - velocity_angle);
            
            % Tính chi phí dựa trên vận tốc (proxemic model)
            if abs(angle_diff) <= pi/2
                % Phía trước: kéo dài theo hướng vận tốc
                sigma_x = sigma_base * factor;
                sigma_y = sigma_base;
                cost = gaussian2D_oriented(point(1), point(2), person_pos(1), person_pos(2), amplitude, sigma_x, sigma_y, velocity_angle);
            else
                % Phía sau: không kéo dài
                cost = gaussian2D_oriented(point(1), point(2), person_pos(1), person_pos(2), amplitude, sigma_base, sigma_base, velocity_angle);
            end
            
            % Áp dụng cutoff
            if cost < cutoff
                cost = 0;
            end
            
            % Lấy giá trị tối đa thay vì cộng dồn
            proxemic_cost_total(i, j) = max(proxemic_cost_total(i, j), cost);
        end
    end
end

fprintf('✅ Hoàn thành tính toán proxemic cost!\n');

%% 5. VẼ BIỂU ĐỒ PROXEMIC

%% 6. TẠO BIỂU ĐỒ PROXEMIC COST 3D
figure('Name', 'Proxemic Cost 3D chi tiết', 'Position', [200, 200, 1400, 800]);

% Surface 3D của proxemic cost
subplot(1, 2, 1);
surf(X_grid, Y_grid, proxemic_cost_total, 'EdgeColor', 'none', 'FaceAlpha', 0.8);
colormap(jet);
colorbar('Location', 'eastoutside');
hold on;
plot3(x, y, zeros(size(x)), 'w-', 'LineWidth', 3);

% Vẽ actor tại vị trí hiện tại
h_robot = drawrobot([x(1), y(1), theta(1)], 'white', 3);
if ~isempty(h_robot)
    for j = 1:length(h_robot)
        if ishandle(h_robot(j)) && strcmp(get(h_robot(j), 'Type'), 'line')
            set(h_robot(j), 'LineWidth', 1.5);
        end
    end
end

title('Proxemic Cost 3D Surface', 'FontSize', 16, 'FontWeight', 'bold');
xlabel('X (m)', 'FontSize', 14);
ylabel('Y (m)', 'FontSize', 14);
zlabel('Proxemic Cost', 'FontSize', 14);
view(45, 30);
grid on;

% Contour 2D của proxemic cost
subplot(1, 2, 2);
contourf(X_grid, Y_grid, proxemic_cost_total, 25, 'LineStyle', 'none');
colormap(jet);
colorbar('Location', 'eastoutside');
hold on;
plot(x, y, 'w-', 'LineWidth', 3);

% Vẽ actor tại vị trí hiện tại
h_robot = drawrobot([x(1), y(1), theta(1)], 'white', 3);
if ~isempty(h_robot)
    for j = 1:length(h_robot(j))
        if ishandle(h_robot(j)) && strcmp(get(h_robot(j), 'Type'), 'line')
            set(h_robot(j), 'LineWidth', 1.5);
        end
    end
end

title('Proxemic Cost Contour 2D', 'FontSize', 16, 'FontWeight', 'bold');
xlabel('X (m)', 'FontSize', 14);
ylabel('Y (m)', 'FontSize', 14);
axis equal;

%% 7. THỐNG KÊ
total_distance = sum(sqrt(diff(x).^2 + diff(y).^2));
total_rotation = theta(end) - theta(1);

fprintf('\n📈 THỐNG KÊ:\n');
fprintf('   Quãng đường: %.2f m\n', total_distance);
fprintf('   Góc quay: %.2f rad (%.1f°)\n', total_rotation, rad2deg(total_rotation));
fprintf('   Vị trí cuối: [%.2f, %.2f]\n', x(end), y(end));
fprintf('   Số điểm dự đoán: %d\n', num_prediction_points);

% Thống kê proxemic cost
max_proxemic_cost = max(proxemic_cost_total(:));
mean_proxemic_cost = mean(proxemic_cost_total(:));
nonzero_points = sum(proxemic_cost_total(:) > 0);

fprintf('\n📊 THỐNG KÊ PROXEMIC COST:\n');
fprintf('   Chi phí tối đa: %.2f\n', max_proxemic_cost);
fprintf('   Chi phí trung bình: %.2f\n', mean_proxemic_cost);
fprintf('   Số điểm có chi phí > 0: %d\n', nonzero_points);
fprintf('   Tỷ lệ điểm có chi phí: %.1f%%\n', 100 * nonzero_points / numel(proxemic_cost_total));

if w ~= 0
    radius_curvature = v / abs(w);
    fprintf('   Bán kính cong: %.2f m\n', radius_curvature);
end

fprintf('\n✅ Hoàn thành mô phỏng proxemic với vận tốc góc!\n');

%% HÀM HỖ TRỢ
function G = gaussian2D_oriented(x, y, x0, y0, A, sigma_x, sigma_y, theta_bearing)
    % GAUSSIAN2D_ORIENTED - Hàm Gaussian 2D có định hướng cho proxemic model
    % 
    % Tham số:
    %   x, y: Tọa độ điểm cần tính
    %   x0, y0: Tọa độ tâm (vị trí người)
    %   A: Biên độ
    %   sigma_x, sigma_y: Phương sai theo trục x, y
    %   theta_bearing: Góc định hướng (radian)
    
    % Tính khoảng cách và góc từ tâm đến điểm
    dx = x - x0;
    dy = y - y0;
    h = sqrt(dx.^2 + dy.^2);
    angle = atan2(dy, dx);
    
    % Chuyển đổi sang hệ tọa độ định hướng
    mx = h .* cos(angle - theta_bearing);
    my = h .* sin(angle - theta_bearing);
    
    % Tính giá trị Gaussian
    f1 = (mx.^2) ./ (2 * sigma_x^2);
    f2 = (my.^2) ./ (2 * sigma_y^2);
    G = A * exp(-(f1 + f2));
end 