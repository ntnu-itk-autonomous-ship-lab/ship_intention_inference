clear all
% filename = "dist_intention_new_Case - 05-09-2018, 10-05-48 - 9PNLJ-60-sec.csv";
% base_filename = "new_Case_LQLVS-60-sec 2.csv"
% base_filename = "new_case_2ZC9Z-60-sec-two-ships.csv"
% base_filename = "new_Case - 01-08-2021, 08-21-29 - AQ5VM-60-sec-two-ships.csv"
% base_filename = "new_Case - 01-15-2020, 09-05-49 - VATEN-60-sec-two-ships.csv"
% base_filename = "new_Case - 01-09-2018, 01-11-37 - RT3LY-60-sec-two-ships-filled.csv"
% base_filename = "new_Case - 01-09-2018, 01-45-02 - 19JNJ-60-sec-two-ships.csv"
% base_filename = "new_Case - 01-11-2019, 02-30-00 - LP84U-60-sec.csv"
% base_filename = "new_Case - 05-09-2018, 10-05-48 - 9PNLJ-60-sec.csv";
% base_filename = "new_Case - 05-26-2019, 20-39-57 - 60GEW-60-sec.csv";
% base_filename = "new_1_Case - 07-09-2019, 05-52-22 - O7LU9-60-sec.csv"; %wierd start
%base_filename = "new_Case - 12-18-2018, 05-11-19 - 4D0M7-60-sec.csv"; %not unmodeled
%base_filename = "new_1_Case - 08-09-2018, 19-12-24 - 4XJ3B-60-sec.csv"; %not unmodeled
%base_filename = "new_1_Case - 06-25-2019, 14-22-43 - OO430-60-sec.csv"; %unmodeled
%base_filename = "new_1_Case - 05-09-2018, 02-26-49 - HQJ86-60-sec.csv"
%base_filename = "new_1_Case - 12-02-2018, 20-10-07 - PW6UL-60-sec.csv"
%base_filename = "new_1_Case - 07-18-2019, 05-46-19 - W6ZUC-60-sec.csv"
%base_filename = "new_1_Case - 09-17-2018, 18-24-32 - 0URFX-60-sec.csv";


% base_filename = "new_Case - 09-09-2018, 14-12-01 - A2WI5-60-sec.csv";
%base_filename = "new_Case - 08-19-2018, 13-13-05 - 3447E-60-sec.csv"
% % base_filename = "new_Case - 03-09-2018, 18-36-52 - 06LIG-60-sec.csv"
% base_filename = "new_Case - 07-10-2019, 00-43-06 - 7VK7W-60-sec.csv"
% base_filename = "new_Case - 06-01-2018, 15-09-18 - CXOPP-60-sec.csv"
% base_filename =  "new_Case - 02-25-2019, 03-47-32 - LGJOO-60-sec.csv"
% base_filename =  "new_Case - 05-25-2018, 03-24-54 - 0PQWV-60-sec.csv"
% base_filename =  "new_Case - 07-09-2019, 21-03-39 - FQSTF-60-sec.csv"
% base_filename =  "new_Case - 02-17-2019, 04-07-49 - FTOY7-60-sec.csv"
% base_filename =  "new_Case - 02-17-2019, 02-40-45 - VFVHD-60-sec.csv"
% base_filename =  "new_Case - 08-26-2019, 04-43-15 - E0ICH-60-sec.csv"
%base_filename = "new_Case - 01-02-2018, 01-05-22 - GP38T-60-sec.csv";


% base_filename = "new_Case - 01-09-2018, 01-45-02 - 19JNJ-60-sec-two-ships.csv"
% base_filename =  "new_Case - 02-17-2019, 02-40-45 - VFVHD-60-sec.csv"
% base_filename = "new_Case - 05-09-2018, 10-05-48 - 9PNLJ-60-sec.csv";
%base_filename =  "new_Case - 05-25-2018, 03-24-54 - 0PQWV-60-sec.csv"
% base_filename = "new_Case - 07-10-2019, 00-43-06 - 7VK7W-60-sec.csv"
% base_filename = "new_Case - 06-01-2018, 15-09-18 - CXOPP-60-sec.csv"
% base_filename = "new_Case - 09-09-2018, 14-12-01 - A2WI5-60-sec.csv";

%base_filename = "new_Case - 01-04-2020, 15-34-37 - 7SWX4-60-sec.csv"; %ot following
%base_filename = "new_Case - 01-12-2018, 03-56-43 - WRNUL-60-sec.csv"; %best correct crossing
%base_filename = "new_Case - 01-10-2018, 11-23-38 - 9W6K5-60-sec.csv"; % ho following colregs
%base_filename = "new_Case - 01-09-2018, 03-55-18 - QZPS3-60-sec.csv";  %best ho not following colregs
%base_filename = "new_Case - 02-01-2018, 15-50-25 - C1401-60-sec.csv"; %best ho following colregs

intention_filename = "nostart_intention_new_1_Case - 12-02-2018, 20-10-07 - PW6UL-60-sec.csv"
%measurement_filename = "measurements_"+base_filename;
%2760

linewidth = 2;
data = readtable(intention_filename);
data{:,"x"} = data{:,"x"}-data{1,"x"};
data{:,"y"} = data{:,"y"}-data{1,"y"};

%measurement_data = readtable(measurement_filename);
%identifier_data = readtable("measurements_identifiers_"+base_filename);

subplot_n_rows = 6;
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
img_pos = [0    0   11.0000    11.2667];
set(gcf,'Position',img_pos);
clf;
hold on;



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
    ax(1) = subplot(subplot_n_rows,subplot_n_cols,[1:subplot_n_cols-1, subplot_n_cols+1:2*subplot_n_cols-1]);
    
    plot(east,north,'DisplayName', shipname_chr,'LineWidth', linewidth);
    %xlim([-3000 3000])
    %ylim([-4200 1200])
    axis equal
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
        ax(2) = subplot(subplot_n_rows,subplot_n_cols,subplot_n_cols);
    else
        ax(3) = subplot(subplot_n_rows,subplot_n_cols,2*subplot_n_cols)
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
    subplot_id = 7;
    %{
    subplot(subplot_n_rows,subplot_n_cols,subplot_id)
    subplot_id=subplot_id+1;
    hold on
    plot(time_vector, measurement_data{rows,"distance_cpa"},'LineWidth', linewidth);
    title('Distance at CPA');
    xlabel('time [s]')
    ylabel('distance [m]')
    xlim([time_vector(1),time_vector(stop_index)]);
    xticks(time_vector(mark_indexes));

    subplot(subplot_n_rows,subplot_n_cols,subplot_id)
    subplot_id=subplot_id+1;
    hold on
    plot(time_vector, data{rows,"will_give_way"},'LineWidth', linewidth);
    title('Will give way');
    xlabel('time [s]')
    ylabel('probability')
    ylim([0 1])
    xlim([time_vector(1),time_vector(stop_index)]);
    xticks(time_vector(mark_indexes));

    subplot(subplot_n_rows,subplot_n_cols,subplot_id)
    hold on;
    subplot_id=subplot_id+1;
    plot(time_vector, data{rows,"situation_started"},'LineWidth', linewidth);
    title('Situation started');
    xlabel('time [s]')
    ylabel('probability')
    ylim([0 1])
    xlim([time_vector(1),time_vector(stop_index)]);
    xticks(time_vector(mark_indexes));    
    
%}
    
    ax(4) = subplot(subplot_n_rows,subplot_n_cols,subplot_id);
    %set(gca,'Position',[0.1 0.45 0.2 0.1]);
    subplot_id=subplot_id+1;
    hold on
    c = data{rows,'colreg_compliant'};
    plot(time_vector,c,'LineWidth', linewidth);
    %daspect([2000 1 1])
    ylim([0 1])
    title('Intention colregs compliant evasive maneuvers')
    xlabel('time [s]')
    ylabel('probability')
    xlim([time_vector(1),time_vector(stop_index)]);
    xticks(time_vector(mark_indexes));
    
    ax(5) = subplot(subplot_n_rows,subplot_n_cols,subplot_id)
    %set(gca,'Position',[0.4 0.45 0.2 0.1]);
    subplot_id=subplot_id+1;
    hold on
    s = data{rows,'good_seamanship'};
    plot(time_vector,s,'LineWidth', linewidth);
    %daspect([2000 1 1])
    ylim([0 1])
    title('Intention good seamanship')
    xlabel('time [s]')
    ylabel('probability')
    xlim([time_vector(1),time_vector(stop_index)]);
    xticks(time_vector(mark_indexes));
    
    ax(6) = subplot(subplot_n_rows,subplot_n_cols,subplot_id)
    % set(gca,'Position',[0.7 0.45 0.2 0.1]);
    subplot_id=subplot_id+1;
    u = data{rows,'unmodeled_behaviour'};
    hold on;
    plot(time_vector,u,'LineWidth', linewidth);
    %daspect([2000 1 1])
    ylim([0 1])
    xlim([time_vector(1),time_vector(stop_index)]);
    title('Intention unmodeled behaviour')
    xlabel('time [s]')
    ylabel('probability')
    xticks(time_vector(mark_indexes));
    
    ax(7) = subplot(subplot_n_rows,subplot_n_cols,subplot_id)
    %set(gca,'Position',[0.13 0.34 0.22 0.08]);
    subplot_id=subplot_id+1;
    pl = data{rows,'priority_lower'};
    hold on;
    plot(time_vector,pl,'LineWidth', linewidth);
    %daspect([2000 1 1])
    ylim([0 1])
    xlim([time_vector(1),time_vector(stop_index)]);
    title({'';'Intention priority lower'})
    xlabel({'time [s]';''})
    ylabel('probability')
    xticks(time_vector(mark_indexes));
    
    ax(8) = subplot(subplot_n_rows,subplot_n_cols,subplot_id)
    %set(gca,'Position',[0.4 0.3 0.2 0.1]);
    subplot_id=subplot_id+1;
    u = data{rows,'priority_similar'};
    hold on;
    plot(time_vector,u,'LineWidth', linewidth);
    %daspect([2000 1 1])
    ylim([0 1])
    xlim([time_vector(1),time_vector(stop_index)]);
    title({'';'Intention priority similar'})
    xlabel({'time [s]';''})
    ylabel('probability')
    xticks(time_vector(mark_indexes));
    
    
    ax(9) = subplot(subplot_n_rows,subplot_n_cols,subplot_id)
    %set(gca,'Position',[0.7 0.3 0.2 0.1]);
    subplot_id=subplot_id+1;
    ph = data{rows,'priority_higher'};
    hold on;
    plot(time_vector,ph,'LineWidth', linewidth);
    %daspect([2000 1 1])
    ylim([0 1])
    xlim([time_vector(1),time_vector(stop_index)]);
    title({'';'Intention priority higher'})
    xlabel({'time [s]';''})
    ylabel('probability')
    xticks(time_vector(mark_indexes));
    
    
    
    
    ax(10) = subplot(subplot_n_rows,subplot_n_cols,subplot_id)
    %set(gca,'Position',[0.4 0.15 0.2 0.1]);
    subplot_id=subplot_id+1;
    ph = data{rows,'is_changing_course'};
    hold on;
    plot(time_vector,ph,'LineWidth', linewidth);
    %daspect([2000 1 1])
    ylim([0 1])
    xlim([time_vector(1),time_vector(stop_index)]);
    title('Is changing course')
    xlabel('time [s]')
    ylabel('probability')
    xticks(time_vector(mark_indexes));
    
    
    
    ax(11) = subplot(subplot_n_rows,subplot_n_cols,subplot_id)
    %set(gca,'Position',[0.13 0.04 0.22 0.08]);
    subplot_id=subplot_id+1;
    crc = data{rows,'has_turned_portwards'};
    hold on;
    plot(time_vector,crc,'LineWidth', linewidth);
    %daspect([2000 1 1])
    ylim([0 1])
    xlim([time_vector(1),time_vector(stop_index)]);
    title('Has turned portwards')
    xlabel('time [s]')
    ylabel('probability')
    xticks(time_vector(mark_indexes));
    
    ax(12) = subplot(subplot_n_rows,subplot_n_cols,subplot_id)
    %set(gca,'Position',[0.4 0.05 0.2 0.1]);
    subplot_id=subplot_id+1;
    crc = data{rows,'has_turned_starboardwards'};
    hold on;
    plot(time_vector,crc,'LineWidth', linewidth);
    %daspect([2000 1 1])
    ylim([0 1])
    xlim([time_vector(1),time_vector(stop_index)]);
    title('Has turned starboardwards')
    xlabel('time [s]')
    ylabel('probability')
    xticks(time_vector(mark_indexes));
    
    ax(13) = subplot(subplot_n_rows,subplot_n_cols,subplot_id)
    %set(gca,'Position',[0.13 0.2 0.22 0.08]);
    subplot_id=subplot_id+1;
    ph = data{rows,'change_in_speed'};
    hold on;
    plot(time_vector,ph,'LineWidth', linewidth);
    %daspect([2000 1 1])
    ylim([0 1])
    xlim([time_vector(1),time_vector(stop_index)]);
    title('Change in speed')
    xlabel('time [s]')
    ylabel('probability')
    xticks(time_vector(mark_indexes));
    
    ax(14) = subplot(subplot_n_rows,subplot_n_cols,subplot_id)
    subplot_id=subplot_id+1;
    crc = data{rows,'current_risk_of_collision'};
    hold on;
    plot(time_vector,crc,'LineWidth', linewidth);
    %daspect([2000 1 1])
    ylim([0 1])
    xlim([time_vector(1),time_vector(stop_index)]);
    title('Current risk of collision')
    xlabel('time [s]')
    ylabel('probability')
    xticks(time_vector(mark_indexes));
    
    
    ax (15) = subplot(subplot_n_rows,subplot_n_cols,subplot_id)
    %set(gca,'Position',[0.7 0.15 0.2 0.1]);
    subplot_id=subplot_id+1;
    rc = data{rows,'risk_of_collision'};
    hold on;
    plot(time_vector,rc,'LineWidth', linewidth);
    %daspect([2000 1 1])
    ylim([0 1])
    xlim([time_vector(1),time_vector(stop_index)]);
    title('Risk of collision')
    xlabel('time [s]')
    ylabel('probability')
    xticks(time_vector(mark_indexes));
    
    
    
    
   

end
%%

subplot(subplot_n_rows,subplot_n_cols,[1:2, 4:5]);

has_added_legend = false;
row_nr = find(data{:,'start'}) 
step_nr = (row_nr+1)/2-1;
for i = 1:length(shipnames)
    j=0;
    if(has_added_legend)
        plot(xs(i,1),ys(i,1), 'X', 'LineWidth',3, 'Color', '#77AC30','HandleVisibility','off');
        plot(xs(i,step_nr),ys(i,step_nr), 'X', 'LineWidth',4, 'Color', '#FF00FF','HandleVisibility','off');
    else
        plot(xs(i,1),ys(i,1), 'X', 'LineWidth',3, 'Color', '#77AC30', 'DisplayName', 'Startpoint');
        has_added_legend =true;
        plot(xs(i,step_nr),ys(i,step_nr), 'X', 'LineWidth',4, 'Color', '#FF00FF', 'DisplayName', 'New Startpoint');
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

%set(ax(1),'Position',[0.11 0.48 0.54 0.67]);
set(ax(2),'Position',[0.7 0.85 0.2 0.1]);
set(ax(3),'Position',[0.7 0.68 0.2 0.1]);
set(ax(4),'Position',[0.12 0.52 0.2 0.07]);
set(ax(5),'Position',[0.42 0.52 0.2 0.07]);
set(ax(6),'Position',[0.7 0.52 0.2 0.07]);
set(ax(7),'Position',[0.12 0.37 0.2 0.07]);
set(ax(8),'Position',[0.42 0.37 0.2 0.07]);
set(ax(9),'Position',[0.7 0.37 0.2 0.07]);
set(ax(10),'Position',[0.12 0.22 0.2 0.07]);
set(ax(11),'Position',[0.42 0.22 0.2 0.07]);
set(ax(12),'Position',[0.7 0.22 0.2 0.07]);
set(ax(13),'Position',[0.12 0.08 0.2 0.07]);
set(ax(14),'Position',[0.42 0.08 0.2 0.07]);
set(ax(15),'Position',[0.7 0.08 0.2 0.07]);


%%
%set(gcf,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[img_pos(3), img_pos(4)])
%possfh = get(sfh,'Position')
%set(gcf, 'Position', possfh + [0 0 10 0]);



%print(gcf,base_filename+'.pdf','-dpdf','-r0')
%%
%{
figure(2);
clf;
hold on;
for i = 1:length(shipnames)
    shipname = shipnames(i);
    shipname_chr = "Ship " + int2str(i);
    rows = find(data{:,'mmsi'}==shipname);

    for j=1:17
        subplot(4,5,j);
        hold on;
        plot(time_vector, measurement_data{rows,j+2}, 'LineWidth', linewidth);
        xlim([time_vector(1),time_vector(stop_index)]);
        title(measurement_data.Properties.VariableNames(j+2), 'Interpreter', 'none')
        xticks(time_vector(mark_indexes));
    end
end


%%
figure(3);
clf;
hold on;
topics = ["distance_risk_of_collision", "distance_risk_of_collision_front", "has_turned_portwards", "has_turned_starboardwards", "stands_on_correct", "ample_time_acceptable", "safe_distance_front_acceptable", "safe_distance_acceptable", "safe_distance_midpoint_acceptable", "is_pre_ample_time", "safe_distance", "safe_distance_to_midpoint", "gives_way_correct", "stand_on_role", "change_in_course_port", "change_in_course_starboard", "change_in_speed_lower", "change_in_speed_higher", "mean_initial_course", "max_initial_course", "mean_initial_speed", "max_initial_speed"];
for i = 1:length(shipnames)
    shipname = shipnames(i);
    shipname_chr = "Ship " + int2str(i);
    rows = find(data{:,'mmsi'}==shipname);

    for j=1:22
        subplot(5,5,j);
        hold on;
        plot(time_vector, data{rows,topics(j)}, 'LineWidth', linewidth);
        xlim([time_vector(1),time_vector(stop_index)]);
        title(topics(j), 'Interpreter', 'none')
        xticks(time_vector(mark_indexes));
    end
end
%}
