clc;
close all;
clear all;

% Defining some general parameters
agents=5; % Total number of agents
Q = 10; % assuming square area always
corners = [0,0;0,Q;Q,Q;Q,0]; % Corners of the specified area
saveFlag = false;  % set to true if you wanna save your results else false
plotFlag = true;  % set to true if you wanna plot your results else false
senseRange = 2;   % range of agent's sensor

% data gathering variable definition
pydata = []; % variable to save agent's position data
maxQ = [];
avgQ = [];
iter = [];

% Initializing random initial position of agent near origin
xPose = 0.5*rand([agents,1]);
yPose = 0.5*rand([agents,1]);

xrange = max(corners(:,1));
yrange = max(corners(:,2));
threshold = 0.01; % distance threshold to achieve

%Dividing the area into discrete cells and expressing range in terms of the map
Undist = ones(Q*100,Q*100); % Uncertainity Distribution
tempUndist = Undist;
lcSensor = max(max(corners))/1000;
senseRange = senseRange/lcSensor;
K = 0.5; % Proportionality constant
cycle=1;

% Initializing handles for visualization
if plotFlag
    verCellHandle = zeros(agents,1);
    cellColors = ones(agents,3);
    for i = 1:numel(xPose)
        verCellHandle(i) = patch(xPose(i),yPose(i),cellColors(i,:)*0.5); % use color i  -- no robot assigned yet
        hold on
    end
    pathHandle = zeros(agents,1);
    for i = 1:numel(xPose)
        pathHandle(i) = plot(xPose(i),yPose(i),'-','color',cellColors(i,:)*.9);
    end
    goalHandle = plot(xPose,yPose,'x','linewidth',2);
    currHandle = plot(xPose,yPose,'o','linewidth',1.5);
    titleHandle = title(['o = Robots, x = Goals, Iteration ', num2str(0)]);
end

figure
surf(Undist);
set(surf(Undist),'LineStyle','none');

% Starting search
dist = xrange*ones(agents,1);
while max(max(Undist))>0.1 % do search until all the points in the specified area is below 0.1
    figure(1)
    count = 0;
    maxQ = [maxQ, max(max(Undist))];
    avgQ = [avgQ, mean(mean(Undist))];
    iter = [iter, cycle];
    while max(dist)>threshold 
        tic;
        count = count+1;
        [v,c] = VoronoiBounded(xPose,yPose,corners);

        if plotFlag
            set(currHandle, 'XData', xPose, 'YData', yPose); % plotting agent's current position
            pydata(cycle,count).Px = xPose;
            pydata(cycle,count).Py = yPose;
            for i = 1:numel(xPose)
                xCurrPath = [get(pathHandle(i),'XData'),xPose(i)];
                yCurrPath = [get(pathHandle(i),'YData'),yPose(i)];
                set(pathHandle(i),'XData',xCurrPath,'YData',yCurrPath); % plot current path position
            end
        end

        for i = 1:numel(c) % calculating centroid of voronoi cells
            roi = roipoly(Undist,100*v(c{i},1),100*v(c{i},2));
            grayRoi = mat2gray(roi);
            for m=1:length(Undist)
                for j=1:length(Undist)
                    grayRoi(j,m) = grayRoi(j,m)*tempUndist(j,m);
                end
            end
            Rprops = regionprops(roi, grayRoi, {'WeightedCentroid'});
            xCentroid = Rprops.WeightedCentroid(1,1)/100;
            yCentroid = Rprops.WeightedCentroid(1,2)/100;
            xCentroid = min(xrange,max(0, xCentroid));
            yCentroid = min(yrange,max(0, yCentroid));
            if ~isnan(xCentroid) && inpolygon(xCentroid,yCentroid,corners(:,1),corners(:,2))
                xPose(i) = xPose(i) + K*(xCentroid - xPose(i));
                yPose(i) = yPose(i) + K*(yCentroid - yPose(i));
                dist(i) = sqrt((xCentroid - xPose(i))^2 + (yCentroid - yPose(i))^2);
            end
        end

        if plotFlag
            for i = 1:numel(c) % update Voronoi cells
                set(verCellHandle(i), 'XData',v(c{i},1),'YData',v(c{i},2));
            end

            set(titleHandle,'string',['o = Robots, x = Goals, Cycle', num2str(cycle,'%3d'), ', Iteration', num2str(count,'%3d')]);
            set(goalHandle,'XData',xPose,'YData',yPose); % plot goal position
            axis equal
            axis([0,xrange,0,yrange]);
            drawnow
        end
        time(cycle,count) = toc;
        pydata(cycle,count).time = toc;
    end
    copyUndist = Undist;
    for i = 1:agents
        roi = roipoly(Undist,100*v(c{i},1),100*v(c{i},2));
        grayRoi = mat2gray(roi);
        Rprops = regionprops(roi, grayRoi, {'Centroid','WeightedCentroid'});
        for m=1:length(Undist)
            for j=1:length(Undist)
                xVal = Rprops.WeightedCentroid(1,1);
                yVal = Rprops.WeightedCentroid(1,2);
                dist = sqrt(((m+0.5)-xVal)^2+((j+0.5)-yVal)^2)/100;
                if dist < senseRange
                    copyUndist(j,m) = min(copyUndist(j,m),(1-0.8708+0.074*dist^2));
                end
            end
        end
    end
    for m=1:length(Undist)
        for j=1:length(Undist)
            Undist(j,m) = Undist(j,m)*copyUndist(j,m);
            tempUndist(j,m) = 2*Undist(j,m)*0.074;
        end
    end
    cycle=cycle+1;
    figure(2);
    surf(Undist);
    set(surf(Undist),'LineStyle','none');
end

if saveFlag==true
    json = jsonencode(pydata);
    fid = fopen("agentSetpoint.json","w");
    fprintf(fid,"%s",json);
    fclose(fid);
end

timeSum = sum(time,2);
timeCumsum = cumsum(timeSum);

hold off
figure;
plot(iter, avgQ)
xlabel("Iterations");
ylabel("Average Q");
title("AverageQ vs Iterations");
figure;
plot(iter, maxQ)
xlabel("Iterations");
ylabel("Max Q");
title("MaxQ vs Iterations");
figure;
plot(iter, timeCumsum)
xlabel("Iterations");
ylabel("Cummulative Search Time (seconds)");
title("Cummulative Search Time vs Iterations");
