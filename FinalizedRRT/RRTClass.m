classdef RRTClass
    properties
        robot
        ptCloud
        k
        deltaq
        bias
        stepsize
        display
        repbinary
        FV
        returnpath
        armradius
        success
    end
    methods
        function obj = RRTClass(robot,ptCloud,k,deltaq,bias,stepsize,display,armradius)
            if nargin > 0
                obj.robot = robot;
                obj.ptCloud = ptCloud;
                obj.k = k;
                obj.deltaq = deltaq;
                obj.bias = bias;
                obj.stepsize = stepsize;
                obj.display = display;
                obj.armradius = armradius;
            else
                load('MANUSFile.mat');
                load('oldPtCloudData.mat', 'ptCloud');
                obj.robot = MANUS;
                obj.ptCloud = ptCloud;
                obj.k = 2500;
                obj.deltaq = .04;
                obj.bias = .05;
                obj.stepsize = .05;
                obj.display = 1;
                obj.armradius = .05;
            end
            [obj.repbinary,obj.FV] = genbinarymap(obj);
            [obj.returnpath,obj.success] = RRTPlanPath(obj);
        end
        
        function [model_handle,FV,A,repbinary] = createVoxelMap(obj)
            %downsizes pt cloud
            ptCloudA = pcdownsample(obj.ptCloud,'gridAverage',obj.stepsize);
            d = pcdenoise(ptCloudA);
            A = double(d.Location);
            %Rounds to accuracy of stepsize
            A(:,1:3) = round(A(:,1:3)/obj.stepsize)*obj.stepsize;
            %Deletes duplicates
            A = unique(A,'rows');
            xr = 0:(4/obj.stepsize);
            yr = 0:(4/obj.stepsize);
            zr = 0:(4/obj.stepsize);
            %create meshgrid
            [Xm,Ym,Zm] = meshgrid(xr, yr, zr);
            Xm = reshape(Xm,[],1);
            Ym = reshape(Ym,[],1);
            Zm = reshape(Zm,[],1);
            %mesh array
            m = [Xm,Ym,Zm];
            %Shift A to 0:4 x and 0:4 y, already 0:4 z
            A(:,1:2) = A(:,1:2) + 2;
            %scale A
            A = A/obj.stepsize;
            A = round(A);
            %'stable'
            C = setdiff(m,A,'rows','stable');
            %if it is unique (aka not in A), place 0
            C(:,4) = 0;
            %if it is in A, place a 1
            A(:,4) = 1;
            %combine a and c for a full map with 0 or 1 for positions
            binarymap = [A;C];
            %sort by column 3
            binarymap = sortrows(binarymap,[1,2,3]);
            %shift the binary map .5 in order to place on lines instead of centers
            binarymap(:,1:3) = binarymap(:,1:3) +.5;
            %find indeces where binary map is 1
            wk = (find(binarymap(:,4)==1));
            %create wm from the binary map!
            wm = binarymap(wk,:);
            %set variables to workspace for viewing
            %just binary portion of binarymap
            repb = binarymap(:,4);
            %reshapes repb to mxmxm matrix
            repbinary = reshape(repb,(4/obj.stepsize)+1,(4/obj.stepsize)+1,(4/obj.stepsize)+1,1);
            s = [];
            s(1:size(A,1),1) = obj.stepsize;
            [model_handle, FV] = drawCubes(obj,wm(:,1:3)*obj.stepsize,s);
        end
        
        function [intpath,success] = RRTPlanPath(obj,start,goal)
            success = [];
            if nargin > 1
                %use input start and goal
                qgoal = goal*pi/180;
            else
                %use standard start and random goal
                start = [7.2,7.2,7.2]*pi/180;
                n_ = 1;
                rvals = 2*rand(n_,1)-1;
                elevation = asin(rvals);
                azimuth = 2*pi*rand(n_,1);
                radii = .7*(rand(n_,1).^(1/3));
                [x_,y_,z_] = sph2cart(azimuth,elevation,radii);
                goalrand = [x_,y_,z_];
                goal = transl(goalrand);
                [qjoint,failed] = obj.robot.ikine(goal);
                qgoal = qjoint(:,1:3);
            end
            pathvalue = [];
            if checkConfig(obj,qgoal) == 1 && failed == 0
                if obj.display == 1
                    disp('goal is valid');
                end
                %checks to see if we can go straight to the goal
                pathvalue = directPathCheck(obj,start,qgoal);
                if any(isnan(pathvalue))
                    [~,~,pathvalue] = RRTConnecter(obj,start,qgoal);
                    if ~isempty(pathvalue)
                        if obj.display == 1
                            disp('RRT Path Found!');
                        end
                        success = 2;
                    end
                else
                    if obj.display == 1
                        disp('Direct Path Found!');
                    end
                    success = 1;
                end
            else
                if obj.display == 1
                    disp('Goal is not valid!');
                end
                success = 0;
            end
            if ~isempty(pathvalue)
                xi = smooth(pathvalue(:,1),15);
                yi = smooth(pathvalue(:,2),15);
                zi = smooth(pathvalue(:,3),15);
                Xi = [xi,yi,zi];
                intpath = displayThings(obj,Xi,'blue');
                if obj.display == 1
                    patch(obj.FV,'edgecolor','k','facecolor','g');
                    axis([0 4 0 4 0 4]);
                    axis square;
                    box on;
                    grid on;
                    sizep = size(intpath);
                    intpath = [intpath,zeros(sizep(1),3)];
                    obj.robot.base = transl(2, 2, 1.25);
                    %Always take this amount of time to animate, increase
                    %fps
                    blev  = size(intpath);
                    time = 1;
                    fps = blev(1) / time;
                    obj.robot.plot(intpath,'trail','-','fps', fps,'jointdiam', obj.armradius*2);
                end
            else
                intpath = NaN;
                if obj.display == 1
                    disp('path not found');
                end
                if isempty(success)
                    success = 3;
                end
            end
        end
        
        function intpath = displayThings(obj,path,color)
            x = path(:,1);
            y = path(:,2);
            z = path(:,3);
            qr = [x,y,z];
            intpath = qr;
            if obj.display == 1
                drawArm(obj,qr,color);
            end
        end
    end
    
    methods(Static)
        function node = gennode()
            a = pi;
            b = -pi;
            J = a + (b-a).*rand(1,3);
            node = J(:,1:3);
        end
    end
    
    methods (Access='protected')
        
        function intpath = directPathCheck(obj,qinit,qgoal)
            qx = linspace(qinit(:,1),qgoal(:,1),25);
            qy = linspace(qinit(:,2),qgoal(:,2),25);
            qz = linspace(qinit(:,3),qgoal(:,3),25);
            if abs(qx(1) - qx(2)) > .1
                intpath = NaN;
            elseif abs(qy(1) - qy(2)) > .1
                intpath = NaN;
            elseif abs(qz(1) - qz(2)) > .1
                intpath = NaN;
            else
                path = [qx',qy',qz'];
                sizepath = size(path);
                configValid = zeros(sizepath(1));
                for w = 1:sizepath(1)
                    configValid(w) = checkConfig(obj,path(w,1:3));
                end
                if any(configValid == 0)
                    intpath = NaN;
                else
                    intpath = path;
                end
            end
        end
        
        function [Ta,Tb,pathvalue] = RRTConnecter(obj,qinit,qgoal)
            %% RRT Algorithm
            %RRT_Connect_planner(qinit,qgoal)
            %Ta(qinit),Tb(qgoal)
            %for k = 1 to K
            %   Qrand
            %   if not(Extend(Ta,qrand) = Trapped then
            %       if(connect(tb,qnew) = reached then
            %           return path
            %   swap(ta,tb)
            %return fail
            %
            %Connect(T,q)
            %repeat
            %   S = Extend(T,q)
            %   until not S = Advanced
            %return S
            %%
            %Extend(T,q)
            %qnear = nearest neighbor(q,T)
            %If new_config(q,qnear,qnew) then
            %   T.add_vertex(qnew)
            %   T.add_edge(qnear,qnew)
            %   if qnew = q then
            %        return reached
            %   else return advanced
            %return trapped
            %%
            Ta = PGraph(3);
            Tb = PGraph(3);
            Ta.add_node(qinit);
            Tb.add_node(qgoal);
            pathvalue = [];
            for k_ = 1:obj.k
                qrand = obj.gennode();
                qrand = qrand + (qgoal - qrand)*obj.bias;
                [Ta,Extenda,qnew] = Extend(obj,Ta,qrand);
                if Extenda ~= 0
                    [Tb,Extendb,~,connect] = Extend(obj,Tb,qnew);
                    if Extendb == 1
                        %% Find the path (A*)
                        sizeTa = Ta.n;
                        for i = 1:Tb.n
                            Ta.add_node(Tb.coord(i));
                        end
                        for i = 1:Tb.n
                            p = Tb.neighbours(i)';
                            for j = 1:size(p)
                                Ta.add_edge(i+sizeTa,p(j)+sizeTa);
                            end
                        end
                        connectid = Ta.closest(connect);
                        qnewid = Ta.closest(qnew);
                        Ta.add_edge(qnewid,connectid);
                        
                        vstart = Ta.closest(qinit);
                        vgoal = Ta.closest(qgoal);
                        %find path through the graph using A* search
                        path = Ta.Astar(vstart, vgoal);
                        sizeP = size(path)';
                        pathvalue = NaN(sizeP(1),3);
                        for i = 1:size(path');
                            pathvalue(i,:) = Ta.coord(path(i));
                        end
                        if obj.display == 10
                            figure
                            Ta.plot('NodeSize',1);
                            Ta.highlight_node(vstart);
                            Ta.highlight_node(vgoal);
                            %Ta.highlight_path(path);
                            axis ([-3.14 3.14 -3.14 3.14 -3.14 3.14]);
                        end
                        break; %%
                    end
                end
                %swap Ta and Tb
                Ttmp = Ta;
                Ta = Tb;
                Tb = Ttmp;
            end
            if isempty(pathvalue)
                pathvalue = [];
            end
        end
        
        function [G,result,qnew,connectedLocation] = Extend(obj,G,q)
            % Algorithm BuildRRT
            %   Input: Initial configuration qinit, number of vertices in RRT K,
            %   incremental distance ?q) Output: RRT graph G
            %
            %   G.init(qinit) for k = 1 to K
            %     qrand ? RAND_CONF() qnear ? NEAREST_VERTEX(qrand, G) qnew ?
            %     NEW_CONF(qnear, qrand, ?q) G.add_vertex(qnew) G.add_edge(qnear, qnew)
            %   return G
            collision = 1;
            collided = 0;
            qnearid = G.closest(q);
            qnear = G.coord(qnearid);
            qnear = qnear';
            qnew = qnear + (((q-qnear)/norm(q-qnear))*obj.deltaq);
            if collision == 1
                con = checkConfig(obj,qnew);
                if con == 1
                    G.add_node(qnew);
                    qnewid = G.closest(qnew);
                    G.add_edge(qnearid,qnewid);
                else
                    collided = 1; % trapped
                end
            else
                G.add_node(qnew);
                qnewid = G.closest(qnew);
                G.add_edge(qnearid,qnewid);
                
            end
            if norm(q-qnew) < obj.deltaq
                connectedLocation = qnew;
                result = 1; %reached
            elseif collided == 1
                connectedLocation = NaN;
                result = 0; %trapped
            else
                result = 2; %Extended
                connectedLocation = NaN;
            end
        end
        
        function configValid = checkConfig(obj,J)
            [j1,j2,j3,j4] = armgen(obj,J);
            distancecheck = round(obj.armradius / obj.stepsize);
            j = [j1;j2;j3;j4];
            %Centers the x value
            j(:,1:2) = (j(:,1:2) + 2);
            j(:,3) = j(:,3) + 1.25;
            %Scales the arm to grid
            j = (j/obj.stepsize);
            sizemap = size(obj.repbinary);
            %initializes foundCollision flag
            foundCollision = 0;
            s = sizemap(1);
            for i = 1:3
                linetest = [j(i,:) j(i+1,:)] ;
                indexes = wooRaytrace([s s s],[0 0 0 s s s],linetest);
                for nVoxel = 1:size(indexes,1)
                end
                %Have to flip the output indexes of the matrix!
                %This works properly
                %find matrix index of the linear indexes from raytrace grid
                [I,J,K] = ind2sub(size(obj.repbinary),indexes);
                %% padding section
                % cube vertices
                cubev = [-1 -1 -1;
                    -1  1 -1;
                    1  1 -1;
                    1 -1 -1;
                    -1 -1  1;
                    -1  1  1;
                    1  1  1;
                    1 -1  1];
                val = (distancecheck * cubev);
                Jr = bsxfun(@plus,J,val(1,:));
                Ir = bsxfun(@plus,I,val(2,:));
                Kr = bsxfun(@plus,K,val(3,:));
                %%
                %remap x and y indexes from raytrace grid!
                %check around these IJK values instead of a ton of raytraces?
                repindexes = sub2ind(size(obj.repbinary),Jr,Ir,Kr);
                
                if any(obj.repbinary(repindexes(:)) == 1)
                    foundCollision = 1;
                end
                if foundCollision == 1
                    configValid = 0;
                else
                    configValid = 1;
                end
            end
        end
        
        function [j1,j2,j3,j4] = armgen(obj,J)
            [~,all] = obj.robot.fkine([J,0,0,0]);
            joints = transl(all);
            %Gets rid of duplicates (wrist/end-effector)
            joints = unique(joints,'rows','stable');
            %Reshapes the matrix into 3xM
            joints = reshape(joints,3,[]);
            %Joint locations for drawing always 4x3.
            %Point 1: base all(:,:,1)
            j1 = joints(1,:);
            %Point 2: offset
            j2 = transl(all(:,:,1) * transl([0 0 -.1]));
            j2 = j2';
            %Point 3: array # 2
            j3 = joints(2,:);
            %Point 4: end effector position
            j4 = joints(3,:);
        end
        
        function [repbinary,FV] = genbinarymap(obj)
            [~,FV,~,repbinary] = createVoxelMap(obj);
            [zInd,yInd,xInd]=ind2sub(size(repbinary),find(repbinary));
            reverse = sub2ind(size(repbinary),xInd,yInd,zInd);
            repbinary(:) = 0;
            repbinary(reverse) = 1;
        end
        
        function drawArm(obj,J,color)
            configurations = size(J);
            JointLocations = NaN(4,3,configurations(1));
            for i = 1:configurations(1)
                [~,all] = obj.robot.fkine([J(i,:),0,0,0]);
                %Size of the joint matrix
                %Converts 4x4 matrix into xyz
                joints = transl(all);
                %Gets rid of duplicates (wrist/end-effector)
                joints = unique(joints,'rows','stable');
                %Reshapes the matrix into 3xM
                joints = reshape(joints,3,[]);
                %Joint locations for drawing always 4x3.
                %Point 1: base all(:,:,1)
                JointLocations(1,:,i) = joints(1,:);
                %Point 2: offset
                JointLocations(2,:,i) = transl(all(:,:,1) * transl([0 0 -.1]));
                %Point 3: array # 2
                JointLocations(3,:,i) = joints(2,:);
                %Point 4: end effector position
                JointLocations(4,:,i) = joints(3,:);
                JointLocations(:,1:2,i) = (JointLocations(:,1:2,i) + 2);
                JointLocations(:,3,i) = JointLocations(:,3,i) + 1.25;
                p = plot3(JointLocations(:,1,i),JointLocations(:,2,i),JointLocations(:,3,i));
                p.Color = color;
                hold on
            end
        end
        
        function [model_handle, FV] = drawCubes(obj,center,cubesize)
            %WireCubes plots cubic wireframe according to the cube cener and its size
            %INPUT:
            %size: nX1 vector containing the size of each cubic element
            %centers: nX3 matrix containing the x,y,z coordinates of the cubes
            %OUTPUT:
            %model_handle: an object handle to the plotted model
            NumberOfCubes=size(center,1);
            verticesTemplate=[0 0 0;
                0 1 0;
                1 1 0;
                1 0 0;
                0 0 1;
                0 1 1;
                1 1 1;
                1 0 1];
            facesTemplate=[1 2 3 4;
                5 6 7 8;
                3 4 8 7;
                1 2 6 5;
                2 3 7 6;
                1 4 8 5];
            FV.faces=zeros(NumberOfCubes*6,4);
            FV.vertices=zeros(NumberOfCubes*8,3);
            X = repmat(verticesTemplate-.5,NumberOfCubes,1);
            Y = repmat(reshape(repmat(cubesize',8,1),[],1),1,3);
            Z = reshape(reshape(repmat(center',8,1),[],1),3,[])';
            f = repmat(facesTemplate,NumberOfCubes,1)+8*repmat(reshape(repmat((0:NumberOfCubes-1)',1,6)',[],1),1,4);
            FV.vertices = X.*Y+Z;
            FV.faces = f;
            if obj.display == 0
                model_handle = 0;
            else
                model_handle=patch(FV,'edgecolor','k','facecolor','r');
                model_handle.Visible = 'off';
            end
        end
        
    end
end