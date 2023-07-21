%filename = "risk_dist_intention_new_case_LQLVS-60-sec.csv" %classisk crossing
%filename = "dist_intention_new_case_2ZC9Z-60-sec-two-ships.csv" %Head on
%filename = "risk_dist_intention_new_Case - 01-08-2021, 08-21-29 - AQ5VM-60-sec-two-ships" %overtaking
%filename = "dist_intention_new_Case - 01-15-2020, 09-05-49 - VATEN-60-sec-two-ships" %overtaking
%filename = "dist_intention_new_Case - 01-04-2020, 15-34-37 - 7SWX4-60-sec.csv" %overtaking used project
%filename = "dist_intention_new_Case - 01-09-2018, 01-11-37 - RT3LY-60-sec-two-ships-filled.csv" %head on long rcpa
%filename = "dist_intention_new_Case - 05-26-2019, 20-39-57 - 60GEW-60-sec2.csv"
%filename = "dist_intention_new_Case - 07-25-2019, 10-57-14 - HUHYL-60-sec.csv"
%filename = "dist_intention_new_Case - 07-17-2019, 14-47-50 - VR28B-60-sec2.csv"
%filename = "forgetting_intention_new_Case - 05-26-2019, 20-39-57 - 60GEW-60-sec.csv"
%filename = "dist_intention_new_Case - 05-26-2019, 20-39-57 - 60GEW-60-sec.csv"
%filename = "dist_intention_new_Case - 01-12-2018, 03-56-43 - WRNUL-60-sec.csv"
%filename = "plot_intention_new_2_Case - 09-17-2018, 18-24-32 - 0URFX-60-sec.csv"

filename = "nostart_intention_new_1_Case - 08-09-2018, 19-12-24 - 4XJ3B-60-sec.csv"
%filename = "nostart_intention_new_1_Case - 06-25-2019, 14-22-43 - OO430-60-sec.csv"
%filename = "nostart_intention_new_1_Case - 07-09-2019, 05-52-22 - O7LU9-60-sec.csv"
clf

data = readtable(filename);

%data.Properties.VariableNames = {'mmsi','x','y','time','CR_PS','CR_SS','HO','OT_en','OT_ing','colreg_compliant','good_seamanship','unmodeled_behaviour','priority_lower','priority_similar','priority_higher'};

shipnames = unique(data{:,'mmsi'});
times = unique(data{:,'time'});

figure(1);
hold on;
axis equal;



for i = 1:length(shipnames)
    shipname = shipnames(i);
    shipname_chr = "Ship " + int2str(i);
    rows = find(data{:,'mmsi'}==shipname);
    north = data{rows,'x'};
    east = data{rows,'y'};
    figure(1);
    hold on;
    subplot(2,2,1)

    plot(east,north,'DisplayName', shipname_chr);
    hold on;
    %xlim([5000 13000])
    %ylim([5000 13000])
    %xlim([6000 15000])
    %ylim([6000 15000])
    title('Position')
    xlabel('East')
    ylabel('North')
    legend('Location','Best')
    
    
    
    subplot(4,3,7)
    c = data{rows,'colreg_compliant'};
    t = data{rows,'time'};
    figure(1);
    hold on;
    plot(t,c);
    hold on
    %daspect([2000 1 1])
    ylim([0 1])
    title('Intention colreg compliant')
    xlabel('time')
    ylabel('probability')
    
    subplot(4,3,8)
    s = data{rows,'good_seamanship'};
    t = data{rows,'time'};
    figure(1);
    hold on;
    plot(t,s);
    hold on
    %daspect([2000 1 1])
    ylim([0 1])
    title('Intention good seamanship')
    xlabel('time')
    ylabel('probability')
    
    
    subplot(4,3,9)
    u = data{rows,'unmodeled_behaviour'};
    t = data{rows,'time'};
    figure(1);
    hold on;
    plot(t,u);
    hold on
    %daspect([2000 1 1])
    ylim([0 1])
    title('Intention unmodeled behaviour')
    xlabel('time')
    ylabel('probability')
    
    subplot(4,3,10)
    pl = data{rows,'priority_lower'};
    t = data{rows,'time'};
    figure(1);
    hold on;
    plot(t,pl);
    hold on
    %daspect([2000 1 1])
    ylim([0 1])
    title('Intention priority lower')
    xlabel('time')
    ylabel('probability')
    
    subplot(4,3,11)
    ps = data{rows,'priority_similar'};
    t = data{rows,'time'};
    figure(1);
    hold on;
    plot(t,ps);
    hold on
    %daspect([2000 1 1])
    ylim([0 1])
    title('Intention priority similar')
    xlabel('time')
    ylabel('probability')
    
    subplot(4,3,12)
    ph = data{rows,'priority_higher'};
    t = data{rows,'time'};
    figure(1);
    hold on;
    plot(t,ph);
    hold on
    %daspect([2000 1 1])
    ylim([0 1])
    title('Intention priority higher')
    xlabel('time')
    ylabel('probability')
    
    %{
    subplot(5,3,13)
    rc = data{rows,'distance_risk_of_collision'};
    t = data{rows,'time'};
    figure(1);
    hold on;
    plot(t,rc);
    hold on
    %daspect([2000 1 1])
    ylim([0 1])
    title('Intention distance risk of collision')
    xlabel('time')
    ylabel('probability')
    
    subplot(5,3,14)
    rcf = data{rows,'distance_risk_of_collision_front'};
    t = data{rows,'time'};
    figure(1);
    hold on;
    plot(t,rcf);
    hold on
    %daspect([2000 1 1])
    ylim([0 1])
    title('Intention distance risk of collision_front')
    xlabel('time')
    ylabel('probability')
    %}
    
    
    
    
end







for ti = 1:length(times)
    head_on=0;
    delete(findall(gcf,'type','annotation'))
    annotation('textbox',[0.55, 0.8, 0.4, 0.1],"string","Intention situation:",'EdgeColor','none','FontSize',12,'fontweight', 'bold')
    for i = 1:length(shipnames)
        size = 0.8-0.05*i;
        shipname = shipnames(i);
        rows = find(data{:,'mmsi'}==shipname);
    
        ho = data{rows(ti),'HO'};
        cr_ss = data{rows(ti),'CR_SS'};
        cr_ps = data{rows(ti),'CR_PS'};
        ot_ing = data{rows(ti),'OT_ing'};
        ot_en = data{rows(ti),'OT_en'};


        %count_ho = sum(ho)
        %count_cr_ss = sum(cr_ss)
        %count_cr_ps = sum(cr_ps)
        %count_ot_ing = sum(ot_ing)
        %count_ot_en = sum(ot_en)
    
        if ho > 0.6
            annotation('textbox',[0.55, size, 0.4, 0.1],'string',"Ship " + i + " is Head-On",'EdgeColor','none')
        elseif cr_ss > 0.6
        annotation('textbox',[0.55, size, 0.4, 0.1],'string',"Ship " + i + " is Crossing Starboard side",'EdgeColor','none')
        elseif cr_ps > 0.6
        annotation('textbox',[0.55, size, 0.4, 0.1],'string',"Ship " + i + " is Crossing Port side",'EdgeColor','none')
        elseif ot_ing > 0.6
        annotation('textbox',[0.55, size, 0.4, 0.1],'string',"Ship " + i + " is Overtaking",'EdgeColor','none')
        elseif ot_en > 0.6
        annotation('textbox',[0.55, size, 0.4, 0.1],'string',"Ship " + i + " is Overtaken",'EdgeColor','none')
        else
            annotation('textbox',[0.55, size, 0.4, 0.1],'string',"Ship " + i + " is Uncertain",'EdgeColor','none')
        end
    
    end
    
       
    
    
    time_utc = times(ti);
    rows = find(data{:,'time'}==time_utc);
    north = data{rows,'x'};
    east = data{rows,'y'};
    c = data{rows,'colreg_compliant'};
    s = data{rows,'good_seamanship'};
    u = data{rows,'unmodeled_behaviour'};
    pl = data{rows,'priority_lower'};
    ps = data{rows,'priority_similar'};
    ph = data{rows,'priority_higher'};
    %rc = data{rows,'distance_risk_of_collision'};
    %rcf = data{rows,'distance_risk_of_collision_front'};
    if ti==1
         subplot(2,2,1)
         plot(east,north, 'x', 'LineWidth',3, 'DisplayName', 'Startpoint');
    end
    t = data{rows,'time'};
    subplot(2,2,1)
    h = scatter(east,north,'DisplayName', 'data');
    hold on;
    subplot(4,3,7)
    k = scatter(t,c);
    hold on;
    subplot(4,3,8)
    l = scatter(t,s);
    hold on;
    subplot(4,3,9)
    m = scatter(t,u);
    hold on;
    subplot(4,3,10)
    n = scatter(t,pl);
    hold on;
    subplot(4,3,11)
    o = scatter(t,ps);
    hold on;
    subplot(4,3,12)
    p = scatter(t,ph);
    hold on;
    %{
    subplot(5,3,13)
    q = scatter(t,rc);
    hold on;
    subplot(5,3,14)
    r = scatter(t,rcf);
    hold on;
    %}
    pause(2);
    delete(h)
    delete(k)
    delete(l)
    delete(m)
    delete(n)
    delete(o)
    delete(p)
    %delete(q)
    %delete(r)
      
    
end









