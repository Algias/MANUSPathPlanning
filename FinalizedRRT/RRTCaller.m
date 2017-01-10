function [returnpath,success] = RRTCaller(varargin)
p = inputParser;
addParameter(p,'j',0);
addParameter(p,'xyz',0);
parse(p,varargin{:});
j = p.Results.j;
xyz =  p.Results.xyz;
load('MANUSFile.mat','MANUS');
cartesian = 0;
%% replace this with actual point cloud generation!
load('oldPtCloudData.mat', 'ptCloud'); %%
%%
%create the basic class object with defaults and the point cloud.
if cartesian == 1
tobj = RRTClassCartesian(MANUS,ptCloud,'display',1);
else
tobj = RRTClass(MANUS,ptCloud,'display',1);
end
%find new point cloud!
%ptCloud = PointCloudAquisition();

%random values
if all(j == 0) && all(xyz == 0)
    [returnpath,success] = tobj.findRoute();
    
    %requested start random goal
elseif any(j ~= 0) && all(xyz == 0)
    [returnpath,success] = tobj.findRoute('startj',j);
    
    %standard start and requested goal
elseif all(j == 0) && any(xyz ~= 0)
    [returnpath,success] = tobj.findRoute('goalxyz',xyz);
    
    %Requested start and requested goal
elseif any(j ~= 0) && any(xyz ~= 0)
    [returnpath,success] = tobj.findRoute('startj',j,'goalxyz',xyz);
end
returnpath = returnpath*180/pi;
returnpath(:,2) = returnpath(:,2)*-1;
end