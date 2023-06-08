%filename = "Relative_intention_new_Case - 05-26-2019, 20-39-57 - 60GEW-60-sec.csv"
%filename = "Relative_intention_new_case_2ZC9Z-60-sec-two-ships.csv"
%filename = "Relative_intention_new_Case - 01-08-2021, 08-21-29 - AQ5VM-60-sec-two-ships.csv"
%filename = "Relative_intention_new_Case - 01-15-2020, 09-05-49 - VATEN-60-sec-two-ships.csv"
%filename = "Relative_intention_new_Case_LQLVS-60-sec_same_cake.csv"
%filename = "Relative_intention_new_Case_LQLVS-60-sec.csv"
%filename = "Relative2_intention_new_Case_LQLVS-60-sec.csv"
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
    axis equal;

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
    c = data{rows,'OT_ing'};
    t = data{rows,'time'};
    figure(1);
    hold on;
    plot(t,c);
    hold on
    ylim([0 1])
    title('overtaking')
    xlabel('time')
    ylabel('probability')
    
    subplot(4,3,8)
    d = data{rows,'OT_en'};
    t = data{rows,'time'};
    figure(1);
    hold on;
    plot(t,d);
    hold on
    ylim([0 1])
    title('overtaken')
    xlabel('time')
    ylabel('probability')
    
    subplot(4,3,9)
    e = data{rows,'CR_PS'};
    t = data{rows,'time'};
    figure(1);
    hold on;
    plot(t,e);
    hold on
    ylim([0 1])
    title('crossing port')
    xlabel('time')
    ylabel('probability')
    
    subplot(4,3,10)
    f = data{rows,'CR_SS'};
    t = data{rows,'time'};
    figure(1);
    hold on;
    plot(t,f);
    hold on
    ylim([0 1])
    title('crossing starboard')
    xlabel('time')
    ylabel('probability')
    
    subplot(4,3,11)
    g = data{rows,'HO'};
    t = data{rows,'time'};
    figure(1);
    hold on;
    plot(t,g);
    hold on
    ylim([0 1])
    title('head on')
    xlabel('time')
    ylabel('probability')
    
    
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
    
        if ho > 0.5
            annotation('textbox',[0.55, size, 0.4, 0.1],'string',"Ship " + i + " is Head-On",'EdgeColor','none')
        elseif cr_ss > 0.5
        annotation('textbox',[0.55, size, 0.4, 0.1],'string',"Ship " + i + " is Crossing Starboard side",'EdgeColor','none')
        elseif cr_ps > 0.5
        annotation('textbox',[0.55, size, 0.4, 0.1],'string',"Ship " + i + " is Crossing Port side",'EdgeColor','none')
        elseif ot_ing > 0.5
        annotation('textbox',[0.55, size, 0.4, 0.1],'string',"Ship " + i + " is Overtaking",'EdgeColor','none')
        elseif ot_en > 0.5
        annotation('textbox',[0.55, size, 0.4, 0.1],'string',"Ship " + i + " is Overtaken",'EdgeColor','none')
        else
            annotation('textbox',[0.55, size, 0.4, 0.1],'string',"Ship " + i + " is Uncertain",'EdgeColor','none')
        end
    
    end
       
    
    
    
    
    time_utc = times(ti);
    rows = find(data{:,'time'}==time_utc);
    north = data{rows,'x'};
    east = data{rows,'y'};
   
    %rc = data{rows,'distance_risk_of_collision'};
    %rcf = data{rows,'distance_risk_of_collision_front'};
    if ti==1
         subplot(2,2,1)
         plot(east,north, 'x', 'LineWidth',3, 'DisplayName', 'Startpoint');
         hold on;
    end
    t = data{rows,'time'};
    subplot(2,2,1)
    h = scatter(east,north,'DisplayName', 'data');
    hold on;
    
    pause(2);
    delete(h)
    
      
    
end
