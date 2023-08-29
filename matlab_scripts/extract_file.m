%filename = "case_LQLVS-60-sec.csv" %classisk crossing
%filename = "case_QVYA6-60-sec.csv" %HO/CR veldig close
%filename = "Case - 04-12-2019, 20-10-56 - DOTVP-two-ships-60-sec.csv"
%filename = "case_2ZC9Z-60-sec-two-ships.csv"
%filename = "Case - 01-08-2021, 08-21-29 - AQ5VM-60-sec-two-ships.csv" %overtaking
%filename = "Case - 01-15-2020, 09-05-49 - VATEN-60-sec-two-ships.csv" %overtaking
%filename = "Case - 01-09-2018, 01-11-37 - RT3LY-60-sec-two-ships-filled.csv"
%filename = "Case - 10-01-2020, 08-42-09 - WELX9-60-sec.csv"
%filename = "Case - 07-17-2019, 14-47-50 - VR28B-60-sec.csv"
%filename = "Case - 01-17-2018, 06-26-20 - W4H51-60-sec.csv"
%filename = "Case - 07-17-2019, 14-47-50 - VR28B-60-sec.csv"
%filename = "Case - 05-26-2019, 20-39-57 - 60GEW-60-sec.csv"
%filename = "Case - 01-12-2018, 03-56-43 - WRNUL-60-sec.csv" %crossing
%filename = "Case - 03-02-2019, 14-50-24 - RKGD1-60-sec.csv" %crossing
%filename = "Case - 01-10-2018, 11-23-38 - 9W6K5-60-sec.csv" %head-on correct
%filename = "Case - 01-09-2018, 03-55-18 - QZPS3-60-sec.csv" %head-on wrong
%filename = "Case - 02-01-2018, 15-50-25 - C1401-60-sec.csv" %head-on correct
%filename = "Case - 01-02-2018, 01-05-22 - GP38T-60-sec.csv" %cross wrong
%filename = "Case - 07-09-2019, 05-52-22 - O7LU9-60-sec.csv" %weird start 1
%filename = "Case - 12-18-2018, 05-11-19 - 4D0M7-60-sec.csv" %not unmodeled
%filename = "Case - 08-09-2018, 19-12-24 - 4XJ3B-60-sec.csv" %not unmodeled
%filename = "Case - 06-25-2019, 14-22-43 - OO430-60-sec.csv" %not unmodeled
%filename = "Case - 12-02-2018, 20-10-07 - PW6UL-60-sec.csv" %unmodeled
%filename = "Case - 07-18-2019, 05-46-19 - W6ZUC-60-sec.csv"  %
%filename = "Case - 09-17-2018, 18-24-32 - 0URFX-60-sec.csv"
filename = "Case - 01-08-2021, 08-21-29 - AQ5VM-60-sec-two-ships-radius-300.csv";


lat0 = 59.5;  %must choose a reference point
lon0 = 5.5;   %must choose a reference point
h = 0;          %because we do not care about height
h0 = h;
wgs84 = wgs84Ellipsoid;
spheroid = wgs84;

clf


data1 = readtable(filename);
%shipnames = unique(data{:,'mmsi'})
shipnames = [257062170,259299000]; %choose the mmsis that is wanted

data = [data1(data1.mmsi == shipnames(1), :);data1(data1.mmsi == shipnames(2), :)];

num_ships = length(shipnames);
num_values = (height(data))/num_ships;
%%times = unique(data{:,'date_time_utc'});
lats = zeros(num_values,num_ships);
lons = zeros(num_values,num_ships);
for i = 1:length(shipnames)
    shipname = shipnames(i);
    rows = find(data{:,'mmsi'}==shipname);
    lats(:,i) = data{rows,'lat'};
    lons(:,i) = data{rows,'lon'};
    
end


x = zeros(height(data),1);
y = zeros(height(data),1);

for i = 1:length(shipnames)
    for j = 1:length(lats)
        [yEast,xNorth,zUp] = geodetic2enu(lats(j,i),lons(j,i),h,lat0,lon0,h0,spheroid);
        index = j +(i-1)*num_values;
        y(index) = yEast;
        x(index) = xNorth;
        scatter(yEast,xNorth)
        xlim([0 25000])
        ylim([0 25000])
        hold on
        pause(0.1);
        
    
    end
end

mmsi = data{:,'mmsi'};
sog = data{:,'sog'};
cog = data{:,'cog'};
times = data{:,'date_time_utc'};
land_port = ~isnan(data{:,'dcoast_port'});
land_front = ~isnan(data{:,'dcoast_front'});
land_starboard = ~isnan(data{:,'dcoast_starboard'});
new_data = table(mmsi,times,x,y,sog,cog,land_port,land_front,land_starboard);
new_filename = sprintf('input_ready_%s',filename);
writetable(new_data,new_filename)


