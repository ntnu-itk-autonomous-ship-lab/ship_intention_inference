clear all
% filename = "dist_intention_new_Case - 05-09-2018, 10-05-48 - 9PNLJ-60-sec.csv";
%base_filename = "new_Case - 05-26-2019, 20-39-57 - 60GEW-60-sec.csv";
base_filename = "new_Case - 01-11-2019, 02-30-00 - LP84U-60-sec.csv"
intention_filename ="dist_intention_"+base_filename;
measurement_filename = "measurements_"+base_filename;

linewidth = 2;
data = readtable(intention_filename);
data{:,"x"} = data{:,"x"}-data{1,"x"};
data{:,"y"} = data{:,"y"}-data{1,"y"};

measurement_data = readtable(measurement_filename);

subplot_n_rows = 4;
subplot_n_cols = 4;

%data.Properties.VariableNames = {'mmsi','x','y','time','CR_PS','CR_SS','HO','OT_en','OT_ing','colreg_compliant','good_seamanship','unmodeled_behaviour','priority_lower','priority_similar','priority_higher'};

shipnames = unique(data{:,'mmsi'});
time_vector = unique(data{:,'time'});

data_index_of_passed = find(measurement_data.hasPassed==1,1);
time_of_passed = measurement_data.time(data_index_of_passed);
time_index_of_passed = find(time_vector==time_of_passed,1);
stop_index = time_index_of_passed ;

num_marks  = 6;
mark_indexes = floor(stop_index*(1:num_marks)/(num_marks));

time_vector = time_vector-time_vector(1);

figure(1);
clf;
hold on;
axis equal;


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
    hold on;
    title('Position')
    xlabel('East')
    ylabel('North')
    legend('Location','Best')
    xs(i,:) = east;
    ys(i,:) = north;

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
    xlabel('time')
    ylabel('probability')
    legend('Location','Best')
    ylim([0 1])
    xlim([time_vector(1),time_vector(stop_index)]);
    xticks(time_vector(mark_indexes));

    subplot_id = 9;
    subplot(subplot_n_rows,subplot_n_cols,subplot_id)
    subplot_id=subplot_id+1;
    c = data{rows,'sog'};
    figure(1);
    hold on;
    plot(time_vector,c,'LineWidth', linewidth);
    hold on
    %daspect([2000 1 1])
    title('Speed over ground')
    xlabel('time')
    ylabel('m/s')
    xlim([time_vector(1),time_vector(stop_index)]);
    xticks(time_vector(mark_indexes));
    
    subplot(subplot_n_rows,subplot_n_cols,subplot_id)
    subplot_id=subplot_id+1;
    c = data{rows,'cog'};
    figure(1);
    hold on;
    plot(time_vector,c,'LineWidth', linewidth);
    hold on
    %daspect([2000 1 1])
    title('Course')
    xlabel('time')
    ylabel('rad')
    xlim([time_vector(1),time_vector(stop_index)]);
    xticks(time_vector(mark_indexes));

    subplot(subplot_n_rows,subplot_n_cols,subplot_id)
    subplot_id=subplot_id+1;
    c = data{rows,'colreg_compliant'};
    figure(1);
    hold on;
    plot(time_vector,c,'LineWidth', linewidth);
    hold on
    %daspect([2000 1 1])
    ylim([0 1])
    title('Intention colreg compliant')
    xlabel('time')
    ylabel('probability')
    xlim([time_vector(1),time_vector(stop_index)]);
    xticks(time_vector(mark_indexes));
    
    subplot(subplot_n_rows,subplot_n_cols,subplot_id)
    subplot_id=subplot_id+1;
    s = data{rows,'good_seamanship'};
    figure(1);
    hold on;
    plot(time_vector,s,'LineWidth', linewidth);
    hold on
    %daspect([2000 1 1])
    ylim([0 1])
    title('Intention good seamanship')
    xlabel('time')
    ylabel('probability')
    xlim([time_vector(1),time_vector(stop_index)]);
    xticks(time_vector(mark_indexes));
    
    subplot(subplot_n_rows,subplot_n_cols,subplot_id)
    subplot_id=subplot_id+1;
    u = data{rows,'unmodeled_behaviour'};
    figure(1);
    hold on;
    plot(time_vector,u,'LineWidth', linewidth);
    hold on
    %daspect([2000 1 1])
    ylim([0 1])
    xlim([time_vector(1),time_vector(stop_index)]);
    title('Intention unmodeled behaviour')
    xlabel('time')
    ylabel('probability')
    xticks(time_vector(mark_indexes));

    subplot(subplot_n_rows,subplot_n_cols,subplot_id)
    subplot_id=subplot_id+1;
    pl = data{rows,'priority_lower'};
    figure(1);
    hold on;
    plot(time_vector,pl,'LineWidth', linewidth);
    hold on
    %daspect([2000 1 1])
    ylim([0 1])
    xlim([time_vector(1),time_vector(stop_index)]);
    title('Intention priority lower')
    xlabel('time')
    ylabel('probability')
    xticks(time_vector(mark_indexes));
    
    subplot(subplot_n_rows,subplot_n_cols,subplot_id)
    subplot_id=subplot_id+1;
    ph = data{rows,'priority_higher'};
    figure(1);
    hold on;
    plot(time_vector,ph,'LineWidth', linewidth);
    hold on
    %daspect([2000 1 1])
    ylim([0 1])
    xlim([time_vector(1),time_vector(stop_index)]);
    title('Intention priority higher')
    xlabel('time')
    ylabel('probability')
    xticks(time_vector(mark_indexes));

    subplot(subplot_n_rows,subplot_n_cols,subplot_id)
    subplot_id=subplot_id+1;
    rc = data{rows,'is_risk_of_collision'};
    figure(1);
    hold on;
    plot(time_vector,rc,'LineWidth', linewidth);
    hold on
    %daspect([2000 1 1])
    ylim([0 1])
    xlim([time_vector(1),time_vector(stop_index)]);
    title('Risk of collision')
    xlabel('time')
    ylabel('probability')
    xticks(time_vector(mark_indexes));
end

subplot(subplot_n_rows,subplot_n_cols,[1:subplot_n_cols-1, subplot_n_cols+1:2*subplot_n_cols-1]);
has_added_legend = false;
for i = 1:length(shipnames)
    if(has_added_legend)
        plot(xs(i,1),ys(i,1), 'x', 'LineWidth',3, 'Color', 'red','HandleVisibility','off');
    else
        plot(xs(i,1),ys(i,1), 'x', 'LineWidth',3, 'Color', 'red', 'DisplayName', 'Startpoint');
        has_added_legend =true;
    end

    for mark = mark_indexes
        plot(xs(i,mark),ys(i,mark), 'x', 'LineWidth',1, 'Color', 'black','HandleVisibility','off');
        text(xs(i,mark)+0.005E4,ys(i,mark),"t="+time_vector(mark))
    end
end
%%
subplot_indexes = setdiff(1:subplot_n_cols*subplot_n_rows, [1:subplot_n_cols-1, subplot_n_cols+1:2*subplot_n_cols-1]);
for subplot_index = subplot_indexes
    subplot(subplot_n_rows,subplot_n_cols,subplot_index);
    ylim_prev = ylim;
    for mark = mark_indexes
        plot([time_vector(mark),time_vector(mark)],[-1000,1000], '--','LineWidth',0.5, 'Color', 'black','HandleVisibility','off');
    end
    ylim(ylim_prev);
end
%%

figure(2);
clf;
hold on;
for i = 1:length(shipnames)
    shipname = shipnames(i);
    shipname_chr = "Ship " + int2str(i);
    rows = find(data{:,'mmsi'}==shipname);
    

    for j=1:12
        subplot(3,4,j);
        hold on;
        plot(time_vector, measurement_data{rows,j+2}, 'LineWidth', linewidth);
        xlim([time_vector(1),time_vector(stop_index)]);
        title(measurement_data.Properties.VariableNames(j+2))
        xticks(time_vector(mark_indexes));
    end
end
