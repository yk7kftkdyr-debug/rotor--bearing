function plot_strong_coupled_results(yn, dyn, ddyn, time, F_b_hist, loaded_count_hist, slip_hist, params, modelInfo)
%PLOT_STRONG_COUPLED_RESULTS Plot coupled simulation histories.
% Inputs:
%   yn, dyn, ddyn          - displacement, velocity and acceleration history.
%   time                   - time vector.
%   F_b_hist               - bearing force history [Fx1; Fy1; Fx2; Fy2...].
%   loaded_count_hist      - loaded element count history.
%   slip_hist              - cage slip ratio history.
%   params, modelInfo      - model metadata.

nb = numel(params.bearing);
time_plot = time(:).';

figure('Name','Bearing orbits');
for ib = 1:nb
    [x, y] = relative_xy(yn, params.bearing(ib), modelInfo);
    x = x - sum(x)/numel(x);
    y = y - sum(y)/numel(y);
    subplot(1, nb, ib);
    plot(x, y, 'LineWidth', 1.0);
    grid on;
    axis equal;
    xlabel('x / m');
    ylabel('y / m');
    title(sprintf('Bearing %d orbit', ib));
end

figure('Name','First bearing response');
brg = params.bearing(1);
[x, y] = relative_xy(yn, brg, modelInfo);
[vx, vy] = relative_xy(dyn, brg, modelInfo);
[ax, ay] = relative_xy(ddyn, brg, modelInfo);

subplot(3,1,1);
plot(time_plot, x, time_plot, y, 'LineWidth', 1.0);
grid on; xlim([0 time_plot(end)]);
xlabel('Time / s'); ylabel('Disp. / m'); legend('x','y');
title('Bearing 1 relative displacement');

subplot(3,1,2);
plot(time_plot, vx, time_plot, vy, 'LineWidth', 1.0);
grid on; xlim([0 time_plot(end)]);
xlabel('Time / s'); ylabel('Vel. / (m/s)'); legend('x','y');
title('Bearing 1 relative velocity');

subplot(3,1,3);
plot(time_plot, ax, time_plot, ay, 'LineWidth', 1.0);
grid on; xlim([0 time_plot(end)]);
xlabel('Time / s'); ylabel('Acc. / (m/s^2)'); legend('x','y');
title('Bearing 1 relative acceleration');

figure('Name','Bearing contact forces');
for ib = 1:nb
    subplot(nb,1,ib);
    plot(time_plot, F_b_hist(2*ib-1,:), time_plot, F_b_hist(2*ib,:), 'LineWidth', 1.0);
    grid on; xlim([0 time_plot(end)]);
    xlabel('Time / s'); ylabel('Force / N');
    legend('Fx','Fy');
    title(sprintf('Bearing %d contact force', ib));
end

figure('Name','Loaded elements and cage slip');
subplot(2,1,1);
plot(time_plot, loaded_count_hist, 'LineWidth', 1.0);
grid on; xlim([0 time_plot(end)]);
xlabel('Time / s'); ylabel('Loaded count');
title('Loaded rolling elements');
legend(make_bearing_labels(nb));

subplot(2,1,2);
plot(time_plot, slip_hist, 'LineWidth', 1.0);
grid on; xlim([0 time_plot(end)]);
xlabel('Time / s'); ylabel('Slip ratio');
title('Cage slip trend');
legend(make_bearing_labels(nb));

end

function labels = make_bearing_labels(nb)
labels = cell(1, nb);
for i = 1:nb
    labels{i} = sprintf('Bearing %d', i);
end
end

function [x, y] = relative_xy(history, brg, modelInfo)
irx = 4*brg.rotor_node - 3;
iry = 4*brg.rotor_node - 2;
icx = modelInfo.num_rotor_dof + 4*brg.case_node - 3;
icy = modelInfo.num_rotor_dof + 4*brg.case_node - 2;
x = history(irx,:) - history(icx,:);
y = history(iry,:) - history(icy,:);
end
