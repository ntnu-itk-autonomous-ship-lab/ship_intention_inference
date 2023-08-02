clear
intention_data = readtable('nostart_intention_new_1_Case - 08-09-2018, 19-12-24 - 4XJ3B-60-sec.csv');
data = readtable('nostart_trajectory_new_1_Case - 08-09-2018, 19-12-24 - 4XJ3B-60-sec.csv');

intention_data = readtable('nostart_intention_new_Case - 02-01-2018, 15-50-25 - C1401-60-sec.csv');
data = readtable('nostart_trajectory_new_Case - 02-01-2018, 15-50-25 - C1401-60-sec.csv');

%intention_data = readtable('nostart_intention_new_Case - 01-17-2018, 06-26-20 - W4H51-60-sec.csv');
%data = readtable('nostart_trajectory_new_Case - 01-17-2018, 06-26-20 - W4H51-60-sec.csv');

intention_data = readtable('nostart_intention_new_Case - 01-09-2018, 03-55-18 - QZPS3-60-sec.csv');
data = readtable('nostart_trajectory_new_Case - 01-09-2018, 03-55-18 - QZPS3-60-sec.csv');

unique_traj_ids = unique(data.traj_id);
unique_time_steps = unique(data.time);
unique_mmsi = unique(intention_data.mmsi);

%timesteps
stop_index = length(unique_time_steps);
num_marks  = 6;
mark_indexes = floor(stop_index*(1:num_marks)/(num_marks));

figure()
hold on;
set(gcf,'Units','Inches');
img_pos = [0    0   11.0000    7];
set(gcf,'Position',img_pos);
xlabel('East');
ylabel('North');
title('Trajectories');
axis equal;

%% Plot trajectory
for i = 1:length(unique_mmsi)
    x_coordinates = intention_data(intention_data.mmsi == unique_mmsi(i), :).x;
    y_coordinates = intention_data(intention_data.mmsi == unique_mmsi(i), :).y;
    plot(y_coordinates, x_coordinates, 'LineWidth', 2, 'DisplayName', sprintf('Ship %d', i));
    j = 0;
end

legend('Location', 'best');

%% Plot time marks
for i = 1:length(unique_mmsi)
    x_coordinates = intention_data(intention_data.mmsi == unique_mmsi(i), :).x;
    y_coordinates = intention_data(intention_data.mmsi == unique_mmsi(i), :).y;
    j = 0;
    for mark = mark_indexes
        j=j+1;
        plot(y_coordinates(mark),x_coordinates(mark), 'x', 'LineWidth',1, 'Color', 'black','HandleVisibility','off');
        text(y_coordinates(mark)+0.005E4,x_coordinates(mark),"t"+j)
    end
end

%% Plot predicted trajectories
for t = 1:length(unique_time_steps)
    time = unique_time_steps(t);
    data_at_time = data(data.time == time, :);
    
    h_marker = gobjects(length(unique_mmsi), length(unique_traj_ids));
    h_line = gobjects(length(unique_mmsi), length(unique_traj_ids));
    h_text = gobjects(length(unique_mmsi), length(unique_traj_ids));


    for j = 1:length(unique_mmsi)
        ship_id = unique_mmsi(j);
        data_for_mmsi = data_at_time(data_at_time.mmsi == ship_id, :);

        [max_prob, ~] = max(data_for_mmsi.prob);
        
        for i = 1:length(unique_traj_ids)
            traj_data = data_for_mmsi(data_for_mmsi.traj_id == unique_traj_ids(i), :);
            
            h_marker(j, i) = plot(traj_data.y(1), traj_data.x(1), 'o', 'MarkerSize', 4, 'MarkerFaceColor', 'red', 'MarkerEdgeColor', 'red', 'HandleVisibility','off');
            
            if traj_data.prob == max_prob
                h_line(j, i) = plot(traj_data.y, traj_data.x, '--', 'MarkerSize', 4, 'LineWidth', 2, 'Color', 'red', 'HandleVisibility','off');
            else
                h_line(j, i) = plot(traj_data.y, traj_data.x, '--', 'MarkerSize', 4, 'LineWidth', 0.7, 'Color', 'black', 'HandleVisibility','off');
            end
        
            last_idx = length(traj_data.prob);
            h_text(j, i) = text(traj_data.y(last_idx), traj_data.x(last_idx), sprintf('%.2f', traj_data.prob(last_idx)), ...
                             'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom', 'FontWeight', 'bold', 'HandleVisibility','off');
        end
    end
    
    pause(4.0);
    
    % Delete previous plot objects after each iteration
    if t > 0
        for j = 1:length(unique_mmsi)
            for i = 1:length(unique_traj_ids)
                delete(h_marker(j, i));
                delete(h_line(j, i));
                delete(h_text(j, i));
            end
        end
    end
end