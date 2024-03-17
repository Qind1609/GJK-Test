format long;
clc;

%% Input and pre-process

[lines, count] = readFile('Enc.txt');
objects = processData(lines, count);

figure
% show 100 first objects
for i = 1:100
    lat = objects{1,i}{1,2};
    lon = objects{1,i}{1,3};
    geoplot(lat,lon, "k");
    hold on;
end

%% Test with Singapore object
%object 7th is Singapore island

Sg_lat = objects{1,7}{1,2};
Sg_lon = objects{1,7}{1,3};

testObject(Sg_lat, Sg_lon, 0.1, 0);


%% Test with Coney Island object
Coney_lat = objects{1,9}{1,2};
Coney_lon = objects{1,9}{1,3};
testObject(Coney_lat, Coney_lon, 0.2, 0);
testObject(Coney_lat, Coney_lon, 0.2, 1);
%% Read file
function [lines, count] = readFile(x)
    file = fopen(x, 'r');
    count = 0;
    lines = cell(1);
    while ~feof(file)
        line = fgetl(file);
        if isempty(line) || ~ischar(line)
            continue
        else
            line = strsplit(line, '\t');
            count = count + 1;
            lines{count} =  line;
        end
    
    end
    fclose(file);
end

%% Process data
function objects = processData(lines, count)
    objects = cell(1);
    for i = 1:count
        objects{i}{1} = lines{i}{2};
        lat = [];
        lon = [];
        for j = 7:2:(length(lines{i})-1)
            lat(end+1) = str2double(lines{i}{j});
            lon(end+1) = str2double(lines{i}{j+1});
        end
        objects{i}{end+1} = lat;
        objects{i}{end+1} = lon;
    end
end

%% draw polygons on map
function [] = drawPolygon(poly_list) 
    for i = 1:length(poly_list)
        P = poly_list{i};
        poly_lat = [];
        poly_lon = [];
        for j = 1:length(P)
            poly_lat(end+1) = P(j,1);
            poly_lon(end+1) = P(j,2);
        end
        geoplot(poly_lat, poly_lon, "g");
        hold on;
    end
end

function []= testObject(lat, lon, threshold, mode)
    figure;
    subplot(1,3,1);
    geoplot(lat,lon, "b");
 
    %% Perform simplifying object
    [Obj_simplified, Obj_origin] = polylineSimplify(lat, lon, threshold);

    %% Display object after simplifying
    subplot(1,3,2);
    simplified_lat = Obj_simplified{1,1};
    simplified_lon = Obj_simplified{1,2};
    geoplot(simplified_lat, simplified_lon, "r");

    %% Perform polygon decomposition - convex partitioning 
    convex_polys_list = convexPartitioning(simplified_lat,simplified_lon,mode);

    %% draw polygons on map
    subplot(1,3,3);
    drawPolygon(convex_polys_list);
end