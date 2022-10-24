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
filename = "Case - 05-26-2019, 20-39-57 - 60GEW-60-sec.csv"

lat0 = 57.95;  %must choose a reference point
lon0 = 6.8;   %must choose a reference point
h = 0;          %because we do not care about height
h0 = h;
wgs84 = wgs84Ellipsoid;
spheroid = wgs84;

clf


data1 = readtable(filename);
%shipnames = unique(data{:,'mmsi'})
shipnames = [211688080,257879290] %choose the mmsis that is wanted

data = [data1(data1.mmsi == shipnames(1), :);data1(data1.mmsi == shipnames(2), :)];

num_ships = length(shipnames)
num_values = (height(data))/num_ships
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
new_data = table(mmsi,times,x,y,sog,cog)
new_filename = sprintf('new_%s',filename);
writetable(new_data,new_filename)


