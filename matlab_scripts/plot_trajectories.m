intention_data = readtable('nostart_intention_new_1_Case - 08-09-2018, 19-12-24 - 4XJ3B-60-sec.csv');
data = readtable('nostart_trajectory_new_1_Case - 08-09-2018, 19-12-24 - 4XJ3B-60-sec.csv');

%intention_data = readtable('nostart_intention_new_Case - 02-01-2018, 15-50-25 - C1401-60-sec.csv');
%data = readtable('nostart_trajectory_new_Case - 02-01-2018, 15-50-25 - C1401-60-sec.csv');

%intention_data = readtable('nostart_intention_new_Case - 01-17-2018, 06-26-20 - W4H51-60-sec.csv');
%data = readtable('nostart_trajectory_new_Case - 01-17-2018, 06-26-20 - W4H51-60-sec.csv');

%intention_data = readtable('nostart_intention_new_Case - 01-09-2018, 03-55-18 - QZPS3-60-sec.csv');
%data = readtable('nostart_trajectory_new_Case - 01-09-2018, 03-55-18 - QZPS3-60-sec.csv');

unique_traj_ids = unique(data.traj_id);
unique_time_steps = unique(data.time);
unique_mmsi = unique(intention_data.mmsi);


figure('Position', [-100, -100, 1200, 1000])
set(gcf,'Units','Inches');
img_pos = [0    0   11.0000    11.2667];
set(gcf,'Position',img_pos);
xlabel('East');
ylabel('North');
title('Trajectories');
axis equal;

legend_labels = arrayfun(@(x) sprintf('Ship %d', x), unique_mmsi, 'UniformOutput', false);

for t = 1:length(unique_time_steps)
    time = unique_time_steps(t);
    data_at_time = data(data.time == time, :);
    
    hold off;

    for i = 1:length(unique_mmsi)
        x_coordinates = intention_data(intention_data.mmsi == unique_mmsi(i), :).x;
        y_coordinates = intention_data(intention_data.mmsi == unique_mmsi(i), :).y;
        plot(y_coordinates, x_coordinates, 'LineWidth', 2);
        hold on;
    end

    for j = 1:length(unique_mmsi)
        ship_id = unique_mmsi(j);
        data_for_mmsi = data_at_time(data_at_time.mmsi == ship_id, :);

        [max_prob, ~] = max(data_for_mmsi.prob);
        for i = 1:length(unique_traj_ids)
            traj_data = data_for_mmsi(data_for_mmsi.traj_id == unique_traj_ids(i), :);

            plot(traj_data.y(1), traj_data.x(1), 'o', 'MarkerSize', 4, 'MarkerFaceColor', 'red', 'MarkerEdgeColor', 'red');
            if traj_data.prob == max_prob
                plot(traj_data.y, traj_data.x, '--', 'MarkerSize', 4, 'LineWidth', 2, 'Color', 'red');
            else
                plot(traj_data.y, traj_data.x, '--', 'MarkerSize', 4, 'LineWidth', 0.7, 'Color', 'black');
            end
        end
    end
    legend(legend_labels, 'Location', 'Best');
    pause(4.0); % Pause for a short duration to observe the plot
end
