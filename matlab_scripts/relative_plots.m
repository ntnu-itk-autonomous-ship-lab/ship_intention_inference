%filename = "Relative_intention_new_Case - 05-26-2019, 20-39-57 - 60GEW-60-sec.csv"
%filename = "Relative_intention_new_case_2ZC9Z-60-sec-two-ships.csv"
%filename = "Relative_intention_new_Case - 01-08-2021, 08-21-29 - AQ5VM-60-sec-two-ships.csv"
%filename = "Relative_intention_new_Case - 01-15-2020, 09-05-49 - VATEN-60-sec-two-ships.csv"
%filename = "Relative_intention_new_Case_LQLVS-60-sec_same_cake.csv"
%filename = "tam1Relative_intention_new_Case_LQLVS-60-sec.csv"
%filename = "TamRelative_intention_new_Case - 05-26-2019, 20-39-57 - 60GEW-60-sec.csv"
%filename = "Relative_intention_new_Case_LQLVS-60-sec 2.csv"
%filename = "Relative_intention_new_Case - 05-26-2019, 20-39-57 - 60GEW-60-sec 2.csv"
%filename = "impRelative_intention_new_Case - 05-26-2019, 20-39-57 - 60GEW-60-sec.csv"
%filename = "emilRelative_intention_new_Case - 01-04-2020, 15-34-37 - 7SWX4-60-sec.csv"
%filename = "emilRelative_intention_new_Case - 02-01-2018, 15-50-25 -
%C1401-60-sec.csv"%
%filename = "emil2Relative_intention_new_Case - 01-09-2018, 03-55-18 - QZPS3-60-sec.csv"
clf
filename = "nostart_intention_new_1_Case - 12-02-2018, 20-10-07 - PW6UL-60-sec.csv"
data = readtable(filename);


data{:,"x"} = data{:,"x"}-data{1,"x"};
data{:,"y"} = data{:,"y"}-data{1,"y"};

%measurement_data = readtable(measurement_filename);
%identifier_data = readtable("measurements_identifiers_"+base_filename);

subplot_n_rows = 2;
subplot_n_cols = 3;

shipnames = unique(data{:,'mmsi'});
time_vector = unique(data{:,'time'});

%data_index_of_passed = find(measurement_data.hasPassed==1,1);
%time_of_passed = measurement_data.time(data_index_of_passed);
%time_index_of_passed = find(time_vector==time_of_passed,1);
% stop_index = time_index_of_passed ;
time_vector = time_vector-time_vector(1);
stop_index = length(time_vector);

num_marks  = 6;
mark_indexes = floor(stop_index*(1:num_marks)/(num_marks));

figure(1);


set(gcf,'Units','Inches');
img_pos = [0    0   12.0000    7.8667];
set(gcf,'Position',img_pos);
clf;
hold on;

linewidth = 1;

xs = zeros(length(shipnames),length(time_vector));
ys = zeros(length(shipnames),length(time_vector));

for i = 1:length(shipnames)
    shipname = shipnames(i);
    shipname_chr = "Ship " + int2str(i);
    rows = find(data{:,'mmsi'}==shipname);
    north = data{rows,'x'};
    east = data{rows,'y'};
    figure(1);
    hold on;
    subplot(subplot_n_rows,subplot_n_cols,[1:subplot_n_cols-1, subplot_n_cols+1:2*subplot_n_cols-1]);
    plot(east,north,'DisplayName', shipname_chr,'LineWidth', linewidth);
    %xlim([-7000 3000])
    %ylim([-5000 2000])
    axis equal;
    hold on;
    title('Position')
    xlabel('East')
    ylabel('North')
    legend('Location','Best')
    xs(i,:) = east;
    ys(i,:) = north;

    %{    
    subplot(subplot_n_rows,subplot_n_cols,subplot_n_cols);
    hold on
    c = data{rows,'sog'};
    plot(time_vector,c,'LineWidth', linewidth);
    %daspect([2000 1 1])
    title('Speed over ground')
    xlabel('time [s]')
    ylabel('speed [m/s]')
    xlim([time_vector(1),time_vector(stop_index)]);
    xticks(time_vector(mark_indexes));
    

    subplot(subplot_n_rows,subplot_n_cols,2*subplot_n_cols);
    hold on
    c = data{rows,'cog'};
    plot(time_vector,wrapTo180(rad2deg(c)),'LineWidth', linewidth);
    %daspect([2000 1 1])
    title('Course')
    xlabel('time [s]')
    ylabel('angle [deg]')
    xlim([time_vector(1),time_vector(stop_index)]);
    xticks(time_vector(mark_indexes));
    %}
    if(i==1)    
        subplot(subplot_n_rows,subplot_n_cols,subplot_n_cols);
    else
        subplot(subplot_n_rows,subplot_n_cols,2*subplot_n_cols)
    end
    hold on
    plot(time_vector, data{rows,"HO"}, 'DisplayName', "HO",'LineWidth', linewidth);
    plot(time_vector, data{rows,"CR_PS"}, 'DisplayName', "CR\_PS",'LineWidth', linewidth);
    plot(time_vector, data{rows,"CR_SS"}, 'DisplayName', "CR\_SS",'LineWidth', linewidth);
    plot(time_vector, data{rows,"OT_ing"}, 'DisplayName', "OT\_ing",'LineWidth', linewidth);
    plot(time_vector, data{rows,"OT_en"}, 'DisplayName', "OT\_en",'LineWidth', linewidth);
    title('Colregs situation '+shipname_chr);
    xlabel('time [s]')
    ylabel('probability')
    legend('Location','Best')
    ylim([0 1])
    xlim([time_vector(1),time_vector(stop_index)]);
    xticks(time_vector(mark_indexes));
    
end

subplot(subplot_n_rows,subplot_n_cols,[1:2, 4:5]);
has_added_legend = false;
for i = 1:length(shipnames)
    j=0;
    if(has_added_legend)
        plot(xs(i,1),ys(i,1), 'X', 'LineWidth',3, 'Color', '#77AC30','HandleVisibility','off');
    else
        plot(xs(i,1),ys(i,1), 'X', 'LineWidth',3, 'Color', '#77AC30', 'DisplayName', 'Startpoint');
        has_added_legend =true;
    end

    for mark = mark_indexes
        j=j+1;
        plot(xs(i,mark),ys(i,mark), 'x', 'LineWidth',1, 'Color', 'black','HandleVisibility','off');
        text(xs(i,mark)+0.005E4,ys(i,mark),"t"+j)
    end
end
%%
subplot_indexes = setdiff(1:subplot_n_cols*subplot_n_rows, [1:2,4:5]);
for subplot_index = subplot_indexes
    subplot(subplot_n_rows,subplot_n_cols,subplot_index);
    ylim_prev = ylim;
    for mark = mark_indexes
        plot([time_vector(mark),time_vector(mark)],[-1000,1000], '--','LineWidth',0.5, 'Color', 'black','HandleVisibility','off');
    end
    ylim(ylim_prev);
end