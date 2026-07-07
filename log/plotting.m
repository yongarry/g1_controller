clear all;
fname = "footstep_log.txt";
nCols = 47;

opts = delimitedTextImportOptions("NumVariables", nCols);
opts.DataLines        = [2, Inf];     % '#' 헤더 줄 건너뜀
opts.Delimiter        = "\t";
opts.ExtraColumnsRule = "ignore";     % 끝의 유령 열 제거
opts.EmptyLineRule    = "skip";

base  = ["tick","walking_tick" ...
         "ref_zmp_x","ref_zmp_y","ref_zmp_z" ...
         "target_com_stance_x","target_com_stance_y","target_com_stance_z" ...
         "com_stance_x","com_stance_y","com_stance_z" ...
         "com_global_x","com_global_y","com_global_z" ...
         "lfoot_x","lfoot_y","lfoot_z" ...
         "rfoot_x","rfoot_y","rfoot_z" ...
         "target_com_global_x","target_com_global_y","target_com_global_z"];
qdes  = "q_leg_desired_" + string(0:11);
qmeas = "q_leg_meas_"    + string(0:11);

opts.VariableNames = [base, qdes, qmeas];
opts.VariableTypes = repmat("double", 1, nCols);

T = readtable(fname, opts);

% 끊긴 마지막 행 처리 (뒤쪽이 NaN이면 제거)
if any(ismissing(T(end,:)))
    T(end,:) = [];
end


%% vrp, preview, pelv traj
fig1 = figure;
zmp_des = T{:, startsWith(T.Properties.VariableNames, "ref_zmp")};
com_des = T{:, startsWith(T.Properties.VariableNames, "target_com_stance")};
com_cur = T{:, startsWith(T.Properties.VariableNames, "com_stance")};

for i = 1:3
    subplot(3,1,i);
    plot(zmp_des(:, i));hold on;
    plot(com_des(:, i));
    plot(com_cur(:, i));
end

%% foot traj global
fig2 = figure;
com_global = T{:, startsWith(T.Properties.VariableNames, "com_global")};
lfoot = T{:, startsWith(T.Properties.VariableNames, "lfoot")};
rfoot = T{:, startsWith(T.Properties.VariableNames, "rfoot")};

for i = 1:3
    subplot(3,1,i);
    plot(com_global(:, i)); hold on;
    plot(lfoot(:, i));
    plot(rfoot(:, i));
    grid on;
    title(['Foot Trajectory ' num2str(i)]);
    legend('COM Global', 'Left Foot', 'Right Foot');
end


%% joint traj
q_des  = T{:, startsWith(T.Properties.VariableNames, "q_leg_desired")};
q_meas = T{:, startsWith(T.Properties.VariableNames, "q_leg_meas")};
% Plot joint trajectories subplot on each joints
fig3 = figure;
for i = 1:size(q_des, 2)
    subplot(2, 6, i);
    plot(q_des(:, i)); hold on;
    plot(q_meas(:, i));
    grid on;
    title(['Joint ' num2str(i-1)]);
    legend('Desired', 'Measured');
end
