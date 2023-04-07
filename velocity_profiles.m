%% load data from Driving Scenario Designer
%% add Images to the path
addpath(genpath('Images'));
load('USCityBlock2.mat');
ego_speed = data.ActorSpecifications(1,1).Speed
Actor1_speed = data.ActorSpecifications(1,2).Speed 
Actor2_speed = data.ActorSpecifications(1,3).Speed
Actor3_speed = data.ActorSpecifications(1,4).Speed
Actor4_speed = data.ActorSpecifications(1,5).Speed
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
           % calculate speed vector for ego 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
speedmatrixego = squareform(pdist(ego_speed));
speedstepsego = zeros(length(ego_speed)-1,1);
for i = 2:length(ego_speed)
    speedstepsego(i-1,1) = speedmatrixego(i,i-1);
end
totalspeedego = sum(speedstepsego); % Total speed 
speedbpego = cumsum([0; speedstepsego]); % speed for each graded point
speedgradbpego = linspace(0,totalspeedego,50); % Linearize speed
% linearize speed vectors based on distance
speedegole = interp1(speedbpego,ego_speed,speedgradbpego);
speed_ego_gdps = smooth(speedgradbpego,speedegole);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%            % calculate speed vector for Actor 1
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
speedmatrixactor1 = squareform(pdist(Actor1_speed));
speedstepsactor1 = zeros(length(Actor1_speed)-1,1);
for ia = 2:length(Actor1_speed)
    speedstepsactor1(ia-1,1) = speedmatrixactor1(ia,ia-1);
end
totalspeedactor1 = sum(speedstepsactor1); % Total speed 
speedbpactor1 = cumsum([0; speedstepsactor1]); % speed for each graded point
speedgradbpactor1 = linspace(0,totalspeedactor1,50); % Linearize speed
% linearize speed vectors based on distance
 speedactor1le = interp1(speedbpactor1,Actor1_speed,speedgradbpactor1);
 speed_actor1_gdps = smooth(speedgradbpactor1,speedactor1le);
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%            % calculate speed vector for Actor 2
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
speedmatrixactor2 = squareform(pdist(Actor2_speed));
speedstepsactor2 = zeros(length(Actor2_speed)-1,1);
for ib = 2:length(Actor2_speed)
    speedstepsactor2(ib-1,1) = speedmatrixactor2(ib,ib-1);
end
totalspeedactor2 = sum(speedstepsactor2); % Total speed 
speedbpactor2 = cumsum([0; speedstepsactor2]); % speed for each graded point
speedgradbpactor2 = linspace(0,totalspeedactor2,50); % Linearize speed
% linearize speed vectors based on distance
speedactor2le = interp1(speedbpactor2,Actor2_speed,speedgradbpactor2);
speed_actor2_gdps = smooth(speedgradbpactor2,speedactor2le);
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%            % calculate speed vector for Actor 3
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
speedmatrixactor3 = squareform(pdist(Actor3_speed));
speedstepsactor3 = zeros(length(Actor3_speed)-1,1);
for ic = 2:length(Actor3_speed)
    speedstepsactor3(ic-1,1) = speedmatrixactor3(ic,ic-1);
end
totalspeedactor3 = sum(speedstepsactor3); % Total speed 
speedbpactor3 = cumsum([0; speedstepsactor3]); % speed for each graded point
speedgradbpactor3 = linspace(0,totalspeedactor3,50); % Linearize speed
speedactor3le = interp1(speedbpactor3,Actor3_speed,speedgradbpactor3);
speed_actor3_gdps = smooth(speedgradbpactor3,speedactor3le);
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%            % calculate speed vector for Actor 4
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
speedmatrixactor4 = squareform(pdist(Actor4_speed));
speedstepsactor4 = zeros(length(Actor4_speed)-1,1);
for id1 = 2:length(Actor4_speed)
    speedstepsactor4(id1-1,1) = speedmatrixactor4(id1,id1-1);
end
totalspeedactor4 = sum(speedstepsactor4); % Total speed 
speedbpactor4 = cumsum([0; speedstepsactor4]); % speed for each graded point
speedgradbpactor4 = linspace(0,totalspeedactor4,50); % Linearize speed
speedactor4le = interp1(speedbpactor4,Actor4_speed,speedgradbpactor4);
speed_actor4_gdps = smooth(speedgradbpactor4,speedactor4le);
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% allspeed=vertcat(speedgradbpego1,speedgradbpactor1,speedgradbpactor2, ...
%     speedgradbpactor3,speedgradbpactor4)