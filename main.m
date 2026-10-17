%% 六轴串联机械臂MDH参数辨识主程序（改进版）
close all; clear; clc;

%% ==================== 1. 名义MDH参数设置 ====================
% UR机器人的名义MDH参数
d1 = 0; 
d2 = 0;
d3 = 0.0607;
d4 = -0.408; 
d5 = 0; 
d6 = 0.10667;
a1 = 0;
a2 = 0;
a3 = 0.38; 
a4 = 0;

% 名义MDH参数向量（24个参数）
Alpha_nominal = [0, pi/2, pi, -pi/2, pi/2, pi/2];
A_nominal = [a1, a2, a3, a4, 0, 0];
D_nominal = [d1, d2, d3, d4, d5, d6];
Theta_nominal = [0, 0, pi, 0, 0, 0];
Sigma = [1, 1, 1, 1, 1, 1];

for i = 1:6
    L(i) = Link('alpha',Alpha_nominal(i),'a', A_nominal(i),'d', D_nominal(i));L(i).offset = Theta_nominal(i); L(i).qlim=[-pi, pi];L(i).mdh = 1;
end

robot = SerialLink(L, 'name', 'UR');

DH_Para.Alpha = Alpha_nominal;
DH_Para.A = A_nominal;
DH_Para.D = D_nominal;
DH_Para.Theta = Theta_nominal;
DH_Para.Sigma = Sigma;

%% ==================== 2. 生成/加载测量数据 ====================
% 这里生成包含参数误差的模拟数据
N = 100;  % 增加数据点数量
fprintf('使用 %d 组测量数据进行参数辨识\n', N);

% 假设的真实参数误差（这些是我们要辨识的）
delta_alpha_true = [0.01, 0.02, 0.03, -0.01, -0.02, 0.05] * pi/180;  % 弧度
delta_a_true = [0.001, 0.002, 0.0123, -0.001, -0.002, 0];  % 米
delta_d_true = [0.002, 0.002, 0.004, -0.002, -0.003, -0.004];  % 米
delta_theta_true = [0.02, 0.03, 0.04, -0.02, -0.03, 0] * pi/180;  % 弧度

% 组合成参数向量（24维）
delta_params_true = [delta_alpha_true, delta_a_true, delta_d_true, delta_theta_true];

fprintf('真实参数误差:\n');
fprintf('δα (度): '); fprintf('%.4f ', rad2deg(delta_alpha_true)); fprintf('\n');
fprintf('δa (mm): '); fprintf('%.4f ', delta_a_true*1000); fprintf('\n');
fprintf('δd (mm): '); fprintf('%.4f ', delta_d_true*1000); fprintf('\n');
fprintf('δθ (度): '); fprintf('%.4f ', rad2deg(delta_theta_true)); fprintf('\n\n');

% 生成测量数据
% Q = zeros(N, 6);  % 关节角度
X_measured = zeros(N, 3);  % 测量的末端位置

% 生成多样化的关节角度配置
% rng(42);  % 固定随机种子以便复现

Q = rand(N, 6) * 2 * pi - pi;  %测量到的关节角

for i = 1:N
    % 生成在合理工作空间内的随机关节角度
    % % 确保覆盖机械臂的主要工作空间
    % if i <= N/4
    %     % 第一象限配置
    %     Q(i, :) = [pi/4 + (pi/8)*randn(), -pi/3 + (pi/12)*randn(), ...
    %                -pi/6 + (pi/12)*randn(), pi/2 + (pi/6)*randn(), ...
    %                pi/4 + (pi/12)*randn(), 0 + (pi/6)*randn()];
    % elseif i <= N/2
    %     % 第二象限配置
    %     Q(i, :) = [-pi/4 + (pi/8)*randn(), -pi/6 + (pi/12)*randn(), ...
    %                pi/6 + (pi/12)*randn(), -pi/2 + (pi/6)*randn(), ...
    %                -pi/4 + (pi/12)*randn(), pi/2 + (pi/6)*randn()];
    % elseif i <= 3*N/4
    %     % 中间配置
    %     Q(i, :) = [0 + (pi/4)*randn(), -pi/4 + (pi/6)*randn(), ...
    %                0 + (pi/4)*randn(), 0 + (pi/2)*randn(), ...
    %                0 + (pi/3)*randn(), pi/4 + (pi/4)*randn()];
    % else
    %     % 随机配置
    %     Q(i, :) = [-pi + 2*pi*rand(), -pi/2 + pi*rand(), -pi + 2*pi*rand(), ...
    %                -pi + 2*pi*rand(), -pi + 2*pi*rand(), -pi + 2*pi*rand()];
    % end
    


    % 使用真实参数（名义值+误差）计算末端位置
    alpha_true = Alpha_nominal + delta_alpha_true;
    a_true = A_nominal + delta_a_true;
    d_true = D_nominal + delta_d_true;
    theta_true = Theta_nominal + delta_theta_true;
    
    % 计算正向运动学得到末端位置
    T = eye(4);
    for j = 1:6
        T = T * transformation_mdh(alpha_true(j), a_true(j), d_true(j), ...
                                  theta_true(j), 1, Q(i, j));
    end
    X_measured(i, :) = T(1:3, 4)';
    
    % 添加微小测量噪声（可选）
    % X_measured(i, :) = X_measured(i, :) + 0.0001 * randn(1, 3);
end

fprintf('数据生成完成，开始参数辨识...\n');


q = Q(1,:);  % 提取第一组6个关节的角度（弧度）
% q = [pi/6, pi/2, 0, pi/3, pi/2, 0];
q_deg = rad2deg(q);  % 弧度转换为角度（更直观）

% 清晰展示第一组关节角
disp('==================== 第一组关节角 ====================');
disp(['关节1角度：', num2str(q(1)), ' 弧度 (', num2str(q_deg(1)), ' °)']);
disp(['关节2角度：', num2str(q(2)), ' 弧度 (', num2str(q_deg(2)), ' °)']);
disp(['关节3角度：', num2str(q(3)), ' 弧度 (', num2str(q_deg(3)), ' °)']);
disp(['关节4角度：', num2str(q(4)), ' 弧度 (', num2str(q_deg(4)), ' °)']);
disp(['关节5角度：', num2str(q(5)), ' 弧度 (', num2str(q_deg(5)), ' °)']);
disp(['关节6角度：', num2str(q(6)), ' 弧度 (', num2str(q_deg(6)), ' °)']);
disp('=====================================================');

% 6. 正运动学计算（原示例）
T = robot.fkine(q); 
disp(' ');  % 空行分隔，更易读
disp('末端位姿（x,y,z）：');
disp(T.t); 

% 7. 可视化机械臂（原示例）
robot.plot(q); 
title(['UR机械臂 - 关节角：', num2str(q_deg(1)), '°, ', num2str(q_deg(2)), '°, ...']);


%% ==================== 3. 改进的参数辨识算法 ====================
% 初始化参数误差（全部为0）
delta_params = zeros(1, 24);

% 算法参数
max_iterations = 100;      % 增加最大迭代次数
tolerance = 1e-8;          % 收敛容差
lambda = 0.2;              % 阻尼因子
lambda_factor = 2;         % 阻尼调整因子
min_lambda = 1e-10;
max_lambda = 1e10;

% 存储历史信息
error_history = zeros(max_iterations + 1, 1);
lambda_history = zeros(max_iterations, 1);

% 计算初始误差
initial_error_vec = compute_position_error(delta_params, Alpha_nominal, A_nominal, ...
                                          D_nominal, Theta_nominal, Q, X_measured);
error_history(1) = norm(initial_error_vec) / sqrt(N);  % RMS误差
fprintf('初始RMS位置误差: %.6f m\n', error_history(1));

% Levenberg-Marquardt算法迭代
fprintf('\n开始Levenberg-Marquardt优化...\n');
fprintf('迭代  RMS误差(m)   参数更新范数   lambda\n');
fprintf('----------------------------------------\n');

for iter = 1:max_iterations
    % 1. 计算当前雅可比矩阵和误差
    J = compute_identification_jacobian(delta_params, Alpha_nominal, A_nominal, ...
                                       D_nominal, Theta_nominal, Q);
    current_error_vec = compute_position_error(delta_params, Alpha_nominal, A_nominal, ...
                                              D_nominal, Theta_nominal, Q, X_measured);
    
    % 2. 计算当前RMS误差
    current_error = norm(current_error_vec) / sqrt(N);
    
    % 3. 构建正规方程 (J^T J + λI) δp = -J^T e
    H = J' * J;
    g = J' * current_error_vec;
    
    % 4. 求解参数更新量
    delta_p = -(H + lambda * eye(24)) \ g;
    
    % 5. 尝试更新参数
    new_params = delta_params + delta_p';
    
    % 6. 计算新参数下的误差
    new_error_vec = compute_position_error(new_params, Alpha_nominal, A_nominal, ...
                                          D_nominal, Theta_nominal, Q, X_measured);
    new_error = norm(new_error_vec) / sqrt(N);
    
    % 7. 判断是否接受更新
    if new_error < current_error
        % 接受更新，减小阻尼因子
        delta_params = new_params;
        error_history(iter + 1) = new_error;
        lambda = max(min_lambda, lambda / lambda_factor);
        
        % 检查收敛条件
        param_update_norm = norm(delta_p);
        if param_update_norm < tolerance
            fprintf('%3d:  %.6f      %.6f      %.2e  (收敛)\n', ...
                    iter, new_error, param_update_norm, lambda);
            fprintf('参数更新量小于容差，迭代终止\n');
            break;
        end
    else
        % 拒绝更新，增大阻尼因子
        error_history(iter + 1) = current_error;
        lambda = min(max_lambda, lambda * lambda_factor);
        
        % 如果lambda变得太大，可能无法找到更好的解
        if lambda > max_lambda/10
            fprintf('阻尼因子过大，可能达到局部最优\n');
            break;
        end
    end
    
    % 记录lambda
    lambda_history(iter) = lambda;
    
    % 显示迭代信息
    fprintf('%3d:  %.6f      %.6f      %.2e\n', ...
            iter, error_history(iter + 1), norm(delta_p), lambda);
    
    % 检查误差收敛条件
    if iter > 1 && abs(error_history(iter + 1) - error_history(iter)) < tolerance
        fprintf('误差变化小于容差，迭代终止\n');
        break;
    end
end

% 截断历史记录
actual_iterations = min(iter, max_iterations);
error_history = error_history(1:actual_iterations+1);
lambda_history = lambda_history(1:actual_iterations);

%% ==================== 4. 处理参数耦合 ====================
% MDH参数存在耦合关系，需要进行后处理
fprintf('\n正在处理参数耦合...\n');

% 提取辨识出的参数误差
delta_alpha = delta_params(1:6);
delta_a = delta_params(7:12);
delta_d = delta_params(13:18);
delta_theta = delta_params(19:24);

% 处理d6和theta6的耦合（常见于最后一个关节）
% 注意：这里的具体处理方式取决于您的机械臂结构
% 下面是一个示例处理，您可能需要根据实际情况调整

% 假设theta6和d6存在耦合：实际的d6误差会影响theta6
% 常见的处理方式：将耦合误差分配到最合适的参数上

% 计算辨识后的实际参数
alpha_identified = Alpha_nominal + delta_alpha;
a_identified = A_nominal + delta_a;
d_identified = D_nominal + delta_d;
theta_identified = Theta_nominal + delta_theta;

%% ==================== 5. 辨识结果展示 ====================
fprintf('\n========== 辨识结果 ==========\n');

% 显示真实误差与辨识误差的对比
fprintf('\n真实误差 vs 辨识误差对比:\n');
fprintf('参数  真实误差        辨识误差        绝对误差\n');
fprintf('----------------------------------------------\n');

for i = 1:6
    fprintf('α%d:   %+8.6f°    %+8.6f°    %8.6f°\n', ...
            i, rad2deg(delta_alpha_true(i)), rad2deg(delta_alpha(i)), ...
            rad2deg(abs(delta_alpha_true(i) - delta_alpha(i))));
end

for i = 1:6
    fprintf('a%d:   %+8.6fmm   %+8.6fmm   %8.6fmm\n', ...
            i, delta_a_true(i)*1000, delta_a(i)*1000, ...
            abs(delta_a_true(i) - delta_a(i))*1000);
end

for i = 1:6
    fprintf('d%d:   %+8.6fmm   %+8.6fmm   %8.6fmm\n', ...
            i, delta_d_true(i)*1000, delta_d(i)*1000, ...
            abs(delta_d_true(i) - delta_d(i))*1000);
end

for i = 1:6
    fprintf('θ%d:   %+8.6f°    %+8.6f°    %8.6f°\n', ...
            i, rad2deg(delta_theta_true(i)), rad2deg(delta_theta(i)), ...
            rad2deg(abs(delta_theta_true(i) - delta_theta(i))));
end

% 计算总体误差指标
param_error_norm = norm(delta_params_true - delta_params);
fprintf('\n参数误差范数: %.6f\n', param_error_norm);

%% ==================== 6. 辨识结果验证 ====================
fprintf('\n========== 结果验证 ==========\n');

% 使用辨识参数计算末端位置误差
N_test = 50;  % 测试数据点数
Q_test = zeros(N_test, 6);
X_test_true = zeros(N_test, 3);
X_test_identified = zeros(N_test, 3);
position_errors = zeros(N_test, 1);

Q_test = rand(N_test, 6) * 2 * pi - pi;

for i = 1:N_test
    % 生成测试关节角度（不同于训练集）
    % Q_test(i, :) = [-pi + 2*pi*rand(), -pi/2 + pi*rand(), -pi + 2*pi*rand(), ...
    %                 -pi + 2*pi*rand(), -pi + 2*pi*rand(), -pi + 2*pi*rand()];
    
    % 使用真实参数计算位置
    T_true = eye(4);
    for j = 1:6
        T_true = T_true * transformation_mdh(Alpha_nominal(j)+delta_alpha_true(j), ...
                                           A_nominal(j)+delta_a_true(j), ...
                                           D_nominal(j)+delta_d_true(j), ...
                                           Theta_nominal(j)+delta_theta_true(j), ...
                                           1, Q_test(i, j));
    end
    X_test_true(i, :) = T_true(1:3, 4)';
    fprintf('\n真实参数的第 %d 个验证点:\n', i);
    disp(X_test_true(i, :));
    
    % 使用辨识参数计算位置
    T_ident = eye(4);
    for j = 1:6
        T_ident = T_ident * transformation_mdh(alpha_identified(j), ...
                                             a_identified(j), ...
                                             d_identified(j), ...
                                             theta_identified(j), ...
                                             1, Q_test(i, j));
    end
    X_test_identified(i, :) = T_ident(1:3, 4)';
     fprintf('\n辨识参数的第 %d 个验证点:\n', i);
    disp(X_test_identified(i, :));
    
    % 计算位置误差
    position_errors(i) = norm(X_test_true(i, :) - X_test_identified(i, :));
end

% 统计误差
mean_error = mean(position_errors);
max_error = max(position_errors);
std_error = std(position_errors);

fprintf('测试集统计（%d个点）:\n', N_test);
fprintf('平均位置误差: %.6f m (%.3f mm)\n', mean_error, mean_error*1000);
fprintf('最大位置误差: %.6f m (%.3f mm)\n', max_error, max_error*1000);
fprintf('位置误差标准差: %.6f m (%.3f mm)\n', std_error, std_error*1000);

% 使用名义参数的误差作为基准
nominal_errors = zeros(N_test, 1);
for i = 1:N_test
    T_nominal = eye(4);
    for j = 1:6
        T_nominal = T_nominal * transformation_mdh(Alpha_nominal(j), A_nominal(j), ...
                                                 D_nominal(j), Theta_nominal(j), ...
                                                 1, Q_test(i, j));
    end
    X_nominal = T_nominal(1:3, 4)';
    nominal_errors(i) = norm(X_test_true(i, :) - X_nominal);
end

fprintf('\n与名义参数对比:\n');
fprintf('名义参数平均误差: %.6f m (%.3f mm)\n', mean(nominal_errors), mean(nominal_errors)*1000);
fprintf('辨识参数改善率: %.1f%%\n', ...
        (mean(nominal_errors) - mean_error) / mean(nominal_errors) * 100);

%% ==================== 7. 可视化结果 ====================
figure('Position', [100, 100, 1400, 800]);

% 子图1: 误差收敛曲线
subplot(2, 3, 1);
plot(0:actual_iterations, error_history * 1000, 'b-o', 'LineWidth', 2);
xlabel('迭代次数');
ylabel('RMS位置误差 (mm)');
title('误差收敛曲线');
grid on;
hold on;
plot([0, actual_iterations], [mean_error*1000, mean_error*1000], 'r--', 'LineWidth', 1.5);
legend('训练误差', sprintf('测试误差: %.2f mm', mean_error*1000));
xlim([0, actual_iterations]);

% 子图2: 阻尼因子变化
subplot(2, 3, 2);
semilogy(1:actual_iterations, lambda_history, 'r-o', 'LineWidth', 2);
xlabel('迭代次数');
ylabel('阻尼因子 λ');
title('阻尼因子变化');
grid on;
xlim([1, actual_iterations]);

% 子图3: 参数误差对比
subplot(2, 3, 3);
x_indices = 1:24;
bar(x_indices, [delta_params_true; delta_params]');
xlabel('参数索引');
ylabel('参数误差值');
title('参数误差对比');
legend('真实误差', '辨识误差', 'Location', 'best');
grid on;
xticks(1:24);
xticklabels({'α1','α2','α3','α4','α5','α6',...
             'a1','a2','a3','a4','a5','a6',...
             'd1','d2','d3','d4','d5','d6',...
             'θ1','θ2','θ3','θ4','θ5','θ6'});
xtickangle(45);

% 子图4: 测试集误差分布
subplot(2, 3, 4);
histogram(position_errors * 1000, 15, 'FaceColor', 'g', 'FaceAlpha', 0.7);
hold on;
histogram(nominal_errors * 1000, 15, 'FaceColor', 'r', 'FaceAlpha', 0.5);
xlabel('位置误差 (mm)');
ylabel('频次');
title(sprintf('测试集误差分布\n辨识: %.2f±%.2f mm, 名义: %.2f±%.2f mm', ...
      mean_error*1000, std_error*1000, mean(nominal_errors)*1000, std(nominal_errors)*1000));
legend('辨识参数', '名义参数');
grid on;

% 子图5: 末端位置对比（3D）
subplot(2, 3, [5, 6]);
plot3(X_test_true(:, 1), X_test_true(:, 2), X_test_true(:, 3), ...
      'bo', 'MarkerSize', 6, 'DisplayName', '真实位置');
hold on;
plot3(X_test_identified(:, 1), X_test_identified(:, 2), X_test_identified(:, 3), ...
      'r+', 'MarkerSize', 8, 'LineWidth', 1.5, 'DisplayName', '辨识位置');

% 连接对应点
for i = 1:min(20, N_test)
    plot3([X_test_true(i, 1), X_test_identified(i, 1)], ...
          [X_test_true(i, 2), X_test_identified(i, 2)], ...
          [X_test_true(i, 3), X_test_identified(i, 3)], ...
          'k-', 'LineWidth', 0.5);
end

xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('末端位置对比（测试集）');
legend('Location', 'best');
grid on;
axis equal;
view(3);

%% ==================== 8. 保存结果 ====================
% 保存辨识结果
save('mdh_identification_results.mat', 'alpha_identified', 'a_identified', ...
     'd_identified', 'theta_identified', 'delta_params', 'error_history', ...
     'position_errors', 'Q', 'X_measured', 'delta_params_true');

% 保存为文本文件方便查看
fid = fopen('identification_results.txt', 'w');
fprintf(fid, 'MDH参数辨识结果\n');
fprintf(fid, '================\n\n');
fprintf(fid, '辨识时间: %s\n', datestr(now));
fprintf(fid, '数据点数: %d\n', N);
fprintf(fid, '迭代次数: %d\n', actual_iterations);
fprintf(fid, '最终RMS误差: %.6f m (%.3f mm)\n\n', error_history(end), error_history(end)*1000);

fprintf(fid, '辨识出的参数误差:\n');
fprintf(fid, '关节   δα(度)     δa(mm)      δd(mm)     δθ(度)\n');
fprintf(fid, '-------------------------------------------------\n');
for i = 1:6
    fprintf(fid, '%2d    %+7.4f   %+7.4f   %+7.4f   %+7.4f\n', ...
            i, rad2deg(delta_alpha(i)), delta_a(i)*1000, ...
            delta_d(i)*1000, rad2deg(delta_theta(i)));
end

fclose(fid);
fprintf('\n辨识结果已保存到文件\n');

%% ==================== 辅助函数（与之前相同） ====================
function error = compute_position_error(delta_params, Alpha_nom, A_nom, D_nom, Theta_nom, Q, X_measured)
    N = size(Q, 1);
    error = zeros(3*N, 1);
    
    delta_alpha = delta_params(1:6);
    delta_a = delta_params(7:12);
    delta_d = delta_params(13:18);
    delta_theta = delta_params(19:24);
    
    alpha = Alpha_nom + delta_alpha;
    a = A_nom + delta_a;
    d = D_nom + delta_d;
    theta = Theta_nom + delta_theta;
    
    for i = 1:N
        T = eye(4);
        for j = 1:6
            T = T * transformation_mdh(alpha(j), a(j), d(j), theta(j), 1, Q(i, j));
        end
        x_pred = T(1:3, 4);
        error(3*i-2:3*i) = x_pred - X_measured(i, :)';
    end
end

function J = compute_identification_jacobian(delta_params, Alpha_nom, A_nom, D_nom, Theta_nom, Q)
    N = size(Q, 1);
    J = zeros(3*N, 24);
    
    % 使用中心差分法提高精度
    epsilon = 1e-8;  % 较小的步长
    
    % 计算当前位置
    pos_current = compute_all_positions(delta_params, Alpha_nom, A_nom, D_nom, Theta_nom, Q);
    
    for param_idx = 1:24
        % 前向扰动
        delta_plus = delta_params;
        delta_plus(param_idx) = delta_plus(param_idx) + epsilon;
        pos_plus = compute_all_positions(delta_plus, Alpha_nom, A_nom, D_nom, Theta_nom, Q);
        
        % 后向扰动
        delta_minus = delta_params;
        delta_minus(param_idx) = delta_minus(param_idx) - epsilon;
        pos_minus = compute_all_positions(delta_minus, Alpha_nom, A_nom, D_nom, Theta_nom, Q);
        
        % 中心差分
        J(:, param_idx) = (pos_plus - pos_minus) / (2 * epsilon);
    end
end

function positions = compute_all_positions(delta_params, Alpha_nom, A_nom, D_nom, Theta_nom, Q)
    N = size(Q, 1);
    positions = zeros(3*N, 1);
    
    delta_alpha = delta_params(1:6);
    delta_a = delta_params(7:12);
    delta_d = delta_params(13:18);
    delta_theta = delta_params(19:24);
    
    alpha = Alpha_nom + delta_alpha;
    a = A_nom + delta_a;
    d = D_nom + delta_d;
    theta = Theta_nom + delta_theta;
    
    for i = 1:N
        T = eye(4);
        for j = 1:6
            T = T * transformation_mdh(alpha(j), a(j), d(j), theta(j), 1, Q(i, j));
        end
        positions(3*i-2:3*i) = T(1:3, 4);
    end
end