%filename = "case_LQLVS-60-sec.csv" %classisk crossing
%filename = "case_T0UNK-60-sec.csv" %masse parkerte båter, noe head-on
%filename = "case_2ZC9Z-60-sec-two-ships.csv" %head on
%filename = "case_QVYA6-60-sec.csv" %HO/CR veldig close
%filename = "Case - 04-12-2019, 20-10-56 - DOTVP-two-ships-60-sec.csv"
%filename = "Case - 01-04-2020, 15-34-37 - 7SWX4-60-sec 2-two-ships.csv"
%filename = "Case - 01-08-2021, 08-21-29 - AQ5VM-60-sec-two-ships.csv"
%filename = "Case - 01-15-2020, 09-05-49 - VATEN-60-sec-two-ships.csv"
%filename = "Case - 01-09-2018, 01-11-37 - RT3LY-60-sec-two-ships-filled.csv"
%filename = "Case - 08-19-2018, 13-43-19 - 9JQSD-60-sec.csv"

clf

data = readtable(filename);

shipnames = unique(data{:,'mmsi'});
times = unique(data{:,'date_time_utc'});

figure(1);
hold on;
axis equal;
prevlats = [];
prevlons = [];
for i = 1:length(shipnames)
    shipname = shipnames(i);
    rows = find(data{:,'mmsi'}==shipname);
    lats = data{rows,'lat'};
    lons = data{rows,'lon'};
    figure(1);
    hold on;
    plot(lons,lats);
end
for t = 1:length(times)
    time_utc = times(t);
    rows = find(data{:,'date_time_utc'}==time_utc);
    lats = data{rows,'lat'};
    lons = data{rows,'lon'};
    h = scatter(lons,lats);
    hold on;
%     if length(prevlats)
%         for ship = 1:length(prevlats)
%             plot([lons(ship),prevlons(ship)],[lats(ship),prevlats(ship)]);
%             hold on;
%         end
%     end
    title(datestr(time_utc));
    pause(0.5);
    delete(h)
    prevlats = lats;
    prevlons = lons;
end