clear
intention_data = readtable('nostart_intention_new_1_Case - 08-09-2018, 19-12-24 - 4XJ3B-60-sec.csv');
data = readtable('nostart_trajectory_new_1_Case - 08-09-2018, 19-12-24 - 4XJ3B-60-sec.csv');

%intention_data = readtable('nostart_intention_new_Case - 02-01-2018, 15-50-25 - C1401-60-sec.csv');
%data = readtable('nostart_trajectory_new_Case - 02-01-2018, 15-50-25 - C1401-60-sec.csv');

%intention_data = readtable('nostart_intention_new_Case - 01-04-2020, 15-34-37 - 7SWX4-60-sec.csv');
%data = readtable('nostart_trajectory_new_Case - 01-04-2020, 15-34-37 - 7SWX4-60-sec.csv');

%intention_data = readtable('nostart_intention_new_1_Case - 06-25-2019, 14-22-43 - OO430-60-sec.csv');
%data = readtable('nostart_trajectory_new_1_Case - 06-25-2019, 14-22-43 - OO430-60-sec.csv');

%intention_data = readtable('nostart_intention_new_1_Case - 12-02-2018, 20-10-07 - PW6UL-60-sec.csv');
%data = readtable('nostart_trajectory_new_1_Case - 12-02-2018, 20-10-07 - PW6UL-60-sec.csv');

%intention_data = readtable('nostart_intention_new_Case - 01-02-2018, 01-05-22 - GP38T-60-sec.csv');
%data = readtable('nostart_trajectory_new_Case - 01-02-2018, 01-05-22 - GP38T-60-sec.csv');

%intention_data = readtable('nostart_intention_new_Case - 01-15-2020, 09-05-49 - VATEN-60-sec-two-ships.csv');
%data = readtable('nostart_trajectory_new_Case - 01-15-2020, 09-05-49 - VATEN-60-sec-two-ships.csv');

%intention_data = readtable('nostart_intention_new_1_Case - 07-09-2019, 05-52-22 - O7LU9-60-sec.csv');
%data = readtable('nostart_trajectory_new_1_Case - 07-09-2019, 05-52-22 - O7LU9-60-sec.csv');

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

%% Plot time marks and start point
has_added_legend = false;

for i = 1:length(unique_mmsi)
    x_coordinates = intention_data(intention_data.mmsi == unique_mmsi(i), :).x;
    y_coordinates = intention_data(intention_data.mmsi == unique_mmsi(i), :).y;

    step_nr = find(intention_data{intention_data.mmsi == unique_mmsi(i),'start'}); 

    if(has_added_legend)
        plot(y_coordinates(1),x_coordinates(1), 'X', 'LineWidth',3, 'Color', '#77AC30','HandleVisibility','off');
        plot(y_coordinates(step_nr),x_coordinates(step_nr), 'X', 'LineWidth',4, 'Color', '#FF00FF','HandleVisibility','off');
    else
        plot(y_coordinates(1),x_coordinates(1), 'X', 'LineWidth',3, 'Color', '#77AC30', 'DisplayName', 'Startpoint');
        has_added_legend =true;
        plot(y_coordinates(step_nr),x_coordinates(step_nr), 'X', 'LineWidth',4, 'Color', '#FF00FF', 'DisplayName', 'New Startpoint');
    end

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
        
        intention_data_at_mmsi = intention_data(intention_data.mmsi == ship_id, :);

        x_coordinate = intention_data_at_mmsi(intention_data_at_mmsi.time == time, :).x;
        y_coordinate = intention_data_at_mmsi(intention_data_at_mmsi.time == time, :).y;

        for i = 1:length(unique_traj_ids)
            traj_data = data_for_mmsi(data_for_mmsi.traj_id == unique_traj_ids(i), :);
            
            x_traj = [x_coordinate; traj_data.x];
            y_traj = [y_coordinate; traj_data.y];


            h_marker(j, i) = plot(y_traj(1), x_traj(1), 'o', 'MarkerSize', 4, 'MarkerFaceColor', 'red', 'MarkerEdgeColor', 'red', 'HandleVisibility','off');
            
            prob = 0.7 + 2*(traj_data.prob(1)/max_prob);
            h_line(j, i) = plot(y_traj, x_traj, '--', 'MarkerSize', 4, 'LineWidth', prob, 'Color', 'magenta', 'HandleVisibility','off');
        
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