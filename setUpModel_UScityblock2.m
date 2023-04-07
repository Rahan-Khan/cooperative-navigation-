% Model Initialization
% ADT Stanley Controller for US Highway
% Copyright 2021 The MathWorks, Inc.

%% add Images to the path
addpath(genpath('Images'));

%% load the scene data files
% load data from Driving Scenario Designer
load('USCityBlock2.mat');
ego_waypoints = data.ActorSpecifications(1,1).Waypoints;
j_waypoints = data.ActorSpecifications(1,2).Waypoints;
k_waypoints = data.ActorSpecifications(1,3).Waypoints;
l_waypoints = data.ActorSpecifications(1,4).Waypoints;
ego_i_waypoints= data.ActorSpecifications(1,5).Waypoints;
% ego_i_time=data.ActorSpecifications(1,5).EntryTime
x_j=j_waypoints(:,1);
x_k=k_waypoints(:,1);
x_l=l_waypoints(:,1);
x_ego_i=ego_i_waypoints(:,1);
y_j=-j_waypoints(:,2);
y_k=-k_waypoints(:,2);
y_l=-l_waypoints(:,2);
y_ego_i=ego_i_waypoints(:,2);

%% define reference points
x_ego_i = ego_waypoints(:,1);
y_ego_i = ego_waypoints(:,2);

%% define vehicle parameters used in the models
X_o = x_ego_i(1); % initial vehicle position in x direction
Y_o = y_ego_i(1); % initial vehicle position in y direction

%% calculating reference pose vectors
% Based on how far the vehicle travels, the pose is generated using 1-D
% lookup tables.

% calculate distance vector
distancematrix = squareform(pdist(ego_waypoints));
distancesteps = zeros(length(ego_waypoints)-1,1);
for i = 2:length(ego_waypoints)
    distancesteps(i-1,1) = distancematrix(i,i-1);
end
totalDistance = sum(distancesteps); % Total traveled distance
distbp = cumsum([0; distancesteps]); % Distance for each waypoint
gradbp = linspace(0,totalDistance,50); % Linearize distance

% linearize X and Y vectors based on distance
xwaypoints = interp1(distbp,x_ego_i,gradbp);
ywaypoints = interp1(distbp,y_ego_i,gradbp);
y_gdps = smooth(gradbp,ywaypoints);
x_gdps = smooth(gradbp,xwaypoints);

%% calculate curvature vector
curvature = getCurvature(xwaypoints,ywaypoints);

%% calculate theta vector
% theta = orientation angle of the path at reference points 
thetaRef = zeros(length(gradbp),1);
for i = 2:length(gradbp)
    thetaRef(i,1) = atan2d((ywaypoints(i)-ywaypoints(i-1)),(xwaypoints(i)-xwaypoints(i-1)));
end
thetaRefs = smooth(gradbp,thetaRef); % smoothing of theta


%% Removing abrupt changes in theta 

% Please refer to the "README.html" file understand this section with images. 

% The built-in function, "findchangepts" finds abrupt changes in the signal.
% When using atan2, if theta is tending towards |180| degrees, the interpolation might
% generate additional points leading to a fluctuating signal between -180
% degrees and 180 degrees. Hence, we follow these steps to smoothen the
% signal:
% -> Find abrupt changes in the signal using findchangepts
% -> Visualize the plot and find the change points where the change is
% causing significant deviation
% -> Define the regions where the signal has to be replaced by a
% neighboring smooth value
% Note: It is a manual process. So it's recommended to visualize the data and tune the
% changepoints and regions to remove the abrupt changes from the signal
% 
% If there is no fluctuation in theta i.e. if -180<theta<=180, consider removing this section and
% use thetaRefs as input to the Simulink model lookup table block named, "Theta Ref"

idx = 10; % number of change points to be visaulized
[ipt, residual] = findchangepts(thetaRefs, 'Statistic','linear','MaxNumChanges',idx);

%% Uncomment this section to visualize the change points
% figure % figure to visualize the change points
% findchangepts(thetaRefs, 'Statistic','linear','MaxNumChanges',idx)

% select the changepoints 
gradbp0 = ipt(1);
gradbp1 = ipt(8);
gradbp2 = max(ipt);
thetaRefab = zeros(length(gradbp),1); % theta after removing the abrupt changes 
% define the regions and remove the abrupt changes from the signal
for i = 1:length(gradbp)
    if gradbp(i) <= gradbp(gradbp0)
        thetaRefab(i) = thetaRefs(gradbp0);  
    elseif (gradbp(i) >= gradbp(gradbp1))
        thetaRefab(i) = thetaRefs(gradbp1);
    else
        thetaRefab(i) = thetaRefs(i);
    end
end
psi_o = thetaRefab(1)*(pi/180); % initial yaw angle

%% plot to visualize the difference between thetaRefs and thetaRefab
% figure
% plot(gradbp,thetaRefab,'Linewidth',5,'Color','r')
% hold on
% plot(gradbp,thetaRefs,'b')
% xlabel('distance (m)')
% ylabel('theta (deg)')
 
%% create direction vector
direction = ones(length(gradbp),1);

%% Curvature Function

function curvature = getCurvature(x_ego_i,y_ego_i)
% Calculate gradient by the gradient of the X and Y vectors
DX = gradient(x_ego_i);
D2X = gradient(DX);
DY = gradient(y_ego_i);
D2Y = gradient(DY);
curvature = (DX.*D2Y - DY.*D2X) ./(DX.^2+DY.^2).^(3/2);
end
