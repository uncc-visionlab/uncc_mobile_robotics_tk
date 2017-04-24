%  MATLAB Code for Mobile Robot Simulation in ROS
%  Code written for ECGR 6090/8090 Mobile Robot Sensing, Mapping and Exploration
%    Copyright (C) 2017  Andrew Willis
%    This program is free software; you can redistribute it and/or modify
%    it under the terms of the GNU General Public License as published by
%    the Free Software Foundation; either version 3 of the License, or
%    (at your option) any later version.
%    This program is distributed in the hope that it will be useful,
%    but WITHOUT ANY WARRANTY; without even the implied warranty of
%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%    GNU General Public License for more details.
%    You should have received a copy of the GNU General Public License
%    along with this program; if not, write to the Free Software Foundation,
%    Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
classdef WorldBuilder_MATLAB < handle
    properties
        % show console output and debug information
        VERBOSE
        % 3D (X,Y,Z) dimensions (length, width, height) of the wall object
        walldims
        % axis-aligned bounding box (AABB) for the wall object
        % in standard position
        aabb
        % List of polygons associated with objects in the world
        polygonList
        numPolygons
        
        % path in the map
        wayPoints
        
        % Gazebo world builder variables
        BUILD_GAZEBO_WORLD
        gazebo
        gazebo_wall_model

        map_landmark_positions
        map_landmark_colors
        
        landmark_models
        landmark_colors
        landmark_diameter
        
        gazeboSpawnedModels
        
        %point_light_model
        
        tfmgr
        
        % Kobuki robot class
        kobuki
        
        randomLocationButton
        resetSimulationButton
    end
    
    methods (Static)
        function world_mat = demo()
            clear all;
            clc;
            close all;
            hold off;
            global GUI;
            GUI = ROSGUI();
            h = GUI.getFigure('MAP');
            set(h,'Visible','on');
            world_mat = WorldBuilder_MATLAB();
            
            world_mat.consolePrint('Shutting down any active ROS processes....');
            rosshutdown;
            pause(1);
            
            GAZEBO_SIM = false;
            WORLD_MAP_INDEX=3;
            world_mat.BUILD_GAZEBO_WORLD=true;
            
            if (world_mat.BUILD_GAZEBO_WORLD)
                %ipaddress = '10.22.77.34';
                ipaddress = '192.168.11.180';
                %ipaddress = '10.16.30.14';
                if (robotics.ros.internal.Global.isNodeActive==0)
                    world_mat.consolePrint(strcat('Initializing ROS node with master IP ....', ...
                        ipaddress));
                    % REPLACE THIS IP WITH YOUR COMPUTER / HOST IP
                    % You can get your host IP by opening the "cmd" program
                    % (from the "run" dialog) and typing "ipconfig" into the
                    % command prompt. Review the console output and find the
                    % "IPv4 Address" of your network card. These values
                    % should be substituted into the ip address of the
                    % command below.
                    %rosinit(ipaddress,'NodeHost','10.38.48.111');
                    rosinit(ipaddress)
                end
                START_TIME = rostime('now');
                GAZEBO_SIM = true;
                world_gaz = WorldBuilder_Gazebo();
                world_mat.setGazeboBuilder(world_gaz);
                list = getSpawnedModels(world_gaz);
                if (ismember('grey_wall',list))
                    world_mat.BUILD_GAZEBO_WORLD=false;
                end
                %world_gaz.removeAllTemporaryModels();
            end
            
            world_mat.makeMap(WORLD_MAP_INDEX);
            %bwImg = world_mat.mapToBWImage(455,245,[10; 10]);
            %figure(2), imshow(bwImg);
        end
    end
    methods (Static)
        function linkstring = addLink(obj,linktype, param, varargin)
            %addLink - Adds a link to the model
            
            position = [0 0 0];
            orientation = [0 0 0];
            bounce = [];
            color = [];
            
            options = varargin(1:end);
            numOptions = numel(options)/2;
            for k = 1:numOptions
                opt = options{2*k-1};
                val = options{2*k};
                if strcmpi(opt,'bounce')
                    bounce = val;
                end
                if strcmpi(opt,'color')
                    color = val;
                end
                if strcmpi(opt,'position')
                    position = val;
                end
                if strcmpi(opt,'orientation')
                    orientation = val;
                end
            end
            
            inpose = [position orientation];
            
            % Creating the link for the model
            model = obj.ModelObj.getElementsByTagName('model');
            
            linknum = model.item(0).getElementsByTagName('link').getLength;
            link = model.item(0).appendChild(obj.ModelObj.createElement('link'));
            link.setAttribute('name', ['link' num2str(linknum)]);
            
            pose = link.appendChild(obj.ModelObj.createElement('pose'));
            pose.appendChild(obj.ModelObj.createTextNode(num2str(inpose)));
            
            % The collision aspect of the object
            collision = link.appendChild(obj.ModelObj.createElement('collision'));
            collision.setAttribute('name','collision');
            
            % The visual aspect of the object
            visual = link.appendChild(obj.ModelObj.createElement('visual'));
            visual.setAttribute('name','visual');
            collision.appendChild(WorldBuilder_MATLAB.getGeometry(obj,linktype,param));
            visual.appendChild(WorldBuilder_MATLAB.getGeometry(obj,linktype,param));
            
            if ~isempty(bounce)
                collision.appendChild(WorldBuilder_MATLAB.getBounce(obj,bounce));
            end
            
            if ~isempty(color)
                visual.appendChild(WorldBuilder_MATLAB.getColor(obj,color));
            end
            
            linkstring = struct('Name', char(link.getAttribute('name')), 'InitialPose', ...
                inpose);
            obj.Links = [obj.Links; linkstring];
            linkstring = linkstring.Name;
        end
        
        function geom = getGeometry(obj,linktype,param)
            %getGeometry - Adds geometry element to the vision or collision tag
            
            geom = obj.ModelObj.createElement('geometry');
            type = geom.appendChild(obj.ModelObj.createElement(linktype));
            dim2 = [];
            switch linktype
                case 'sphere'
                    dim = type.appendChild(obj.ModelObj.createElement('radius'));
                case 'box'
                    dim = type.appendChild(obj.ModelObj.createElement('size'));
                case 'cylinder'
                    dim2 = type.appendChild(obj.ModelObj.createElement('radius'));
                    dim = type.appendChild(obj.ModelObj.createElement('length'));
            end
            dim.appendChild(obj.ModelObj.createTextNode(num2str(param(1))));
            if ~isempty(dim2)
                dim2.appendChild(obj.ModelObj.createTextNode(num2str(param(2))));
            end
        end
        
        function material = getColor(obj,color)
            %getColor - Creates element for the color of the link material
            
            material = obj.ModelObj.createElement('material');
            ambient = material.appendChild(obj.ModelObj.createElement('ambient'));
            diffuse = material.appendChild(obj.ModelObj.createElement('diffuse'));
            emissive = material.appendChild(obj.ModelObj.createElement('emissive'));
            ambient.appendChild(obj.ModelObj.createTextNode(num2str(color)));
            diffuse.appendChild(obj.ModelObj.createTextNode(num2str(color)));
            emissive.appendChild(obj.ModelObj.createTextNode(num2str(color)));
        end
        
        function surface = getBounce(obj,bounce)
            %getBounce - Creates element for the bounciness of the surface material
            
            surface = obj.ModelObj.createElement('surface');
            bouncetag = surface.appendChild(obj.ModelObj.createElement('bounce'));
            
            rcoeff = bouncetag.appendChild(obj.ModelObj.createElement('restitution_coefficient'));
            rcoeff.appendChild(obj.ModelObj.createTextNode(num2str(bounce(1))));
            
            threshold = bouncetag.appendChild(obj.ModelObj.createElement('threshold'));
            threshold.appendChild(obj.ModelObj.createTextNode(num2str(0)));
            
            contacttag = surface.appendChild(obj.ModelObj.createElement('contact'));
            ode = contacttag.appendChild(obj.ModelObj.createElement('ode'));
            maxvel = ode.appendChild(obj.ModelObj.createElement('max_vel'));
            maxvel.appendChild(obj.ModelObj.createTextNode(num2str(bounce(2))));
        end
    end

    methods
        function obj = WorldBuilder_MATLAB()
            global GUI;
            
            obj.walldims=[7.5,0.2,2.8]';
            obj.aabb=[-3.75 3.75 3.75 -3.75 -3.75;
                0.1 0.1 -0.1 -0.1 0.1];
            obj.numPolygons = 0;
            obj.VERBOSE = 1;
            % Create buttons in the GUI window
            obj.randomLocationButton = uicontrol(GUI.getFigure('MAP'), ...
                'Style', 'pushbutton', ...
                'String','Start/Stop Sim.', ...
                'Units', 'normalized', ...
                'Position', [0.80 0.01 0.2 0.05], ...
                'Callback', @obj.pauseSimulationButtonCallback);
            
            obj.resetSimulationButton = uicontrol(GUI.getFigure('MAP'), ...
                'Style', 'pushbutton', ...
                'String','Reset Simulation', ...
                'Units', 'normalized', ...
                'Position', [0.0 0.01 0.2 0.05], ...
                'Callback', @obj.resetSimulationButtonCallback);
        end
        
        function setGazeboBuilder(obj, gazbuilder)
            obj.gazebo = gazbuilder;
            obj.gazebo_wall_model = ExampleHelperGazeboModel('grey_wall','gazeboDB');
            %obj.point_light_model = ExampleHelperGazeboModel('user_point_light','gazeboDB');
            obj.landmark_colors = [1.0 0.0 0.0;
                0.0 1.0 0.0;
                0.0 0.0 1.0;
                1.0 1.0 0.0;
                0.0 1.0 1.0;
                1.0 0.0 1.0];
                %0.5 0.5 0.0;
                %0.0 0.5 0.5;
                %0.5 0.0 0.5];
            obj.landmark_diameter = 0.1; % 10 cm diameter
            numLandmarkModels = size(obj.landmark_colors,1);
            for modelIdx=1:numLandmarkModels
                nameStr = sprintf('landmark_sdfs/landmark_%03d',modelIdx);
                obj.landmark_models{modelIdx} = ExampleHelperGazeboModel(nameStr);
                if (1==0)
                landmark_model = obj.landmark_models{modelIdx};
                    cyl_height_radius = [0.3 0.02];
                    sphere_radius = 0.05;
                    link1 = WorldBuilder_MATLAB.addLink( landmark_model, ...
                        'cylinder',cyl_height_radius, ...
                        'position',[0,0,0.5*cyl_height_radius(1)], ...
                        'color', [1 1 1 1]);
                    link2 = WorldBuilder_MATLAB.addLink( landmark_model, ...
                        'sphere',sphere_radius, ...
                        'position',[0,0, cyl_height_radius(1)+0.5*sphere_radius], ...
                        'color',[0.7 0 0.2 1]);
                    landmark_model.addJoint( link1, link2, 'revolute', [0 0], ...
                        [0 0 cyl_height_radius(1)]);
                end
            end
        end
        
        function makeMap(world_mat, map_index)
            dir_right=[1,0]';
            dir_up=[0,1]';
            if (map_index == 1)
                pstart=[-3.5,-3.5]';
                pend=pstart+1*world_mat.walldims(1)*dir_right;
                world_mat.makeWall(pstart, pend);
                pstart=pend;
                pend=pstart+1*world_mat.walldims(1)*dir_up;
                world_mat.makeWall(pstart, pend);
                pstart=pend;
                pend=pstart-1*world_mat.walldims(1)*dir_right;
                world_mat.makeWall(pstart, pend);
                pstart=pend;
                pend=pstart-1*world_mat.walldims(1)*dir_up;
                world_mat.makeWall(pstart, pend);
            elseif (map_index == 2)
                pstart=[-20,-10]';
                pend=pstart+6*world_mat.walldims(1)*dir_right;
                world_mat.makeWall(pstart, pend);
                pstart=pend;
                pend=pstart+3*world_mat.walldims(1)*dir_up;
                world_mat.makeWall(pstart, pend);
                pstart=pend;
                pend=pstart+2*world_mat.walldims(1)*dir_right;
                world_mat.makeWall(pstart, pend);
                pstart=pend;
                pend=pstart+1*world_mat.walldims(1)*dir_up;
                world_mat.makeWall(pstart, pend);
                pstart=pend;
                pend=pstart-6*world_mat.walldims(1)*dir_right;
                world_mat.makeWall(pstart, pend);
                pstart=pend;
                pend=pstart-3*world_mat.walldims(1)*dir_up;
                world_mat.makeWall(pstart, pend);
                pstart=pend;
                pend=pstart-2*world_mat.walldims(1)*dir_right;
                world_mat.makeWall(pstart, pend);
                pstart=pend;
                pend=pstart-1*world_mat.walldims(1)*dir_up;
                world_mat.makeWall(pstart, pend);
                pstart=[5,13.5]';
                pend=pstart-2*world_mat.walldims(1)*dir_up;
                world_mat.makeWall(pstart, pend);
                pstart=pend+[1.5*world_mat.walldims(1),0]';
                pend=pstart+2*world_mat.walldims(1)*dir_up;
                world_mat.makeWall(pstart, pend);
            elseif (map_index == 3)
                pstart=[-3.5,-3.5]';
                pend=pstart+1*world_mat.walldims(1)*dir_right;
                world_mat.makeWall(pstart, pend);
                pstart=pend;
                pend=pstart+1*world_mat.walldims(1)*dir_up;
                world_mat.makeWall(pstart, pend);
                pstart=pend;
                pend=pstart-1*world_mat.walldims(1)*dir_right;
                world_mat.makeWall(pstart, pend);
                pstart=pend;
                pend=pstart-1*world_mat.walldims(1)*dir_up;
                world_mat.makeWall(pstart, pend);
                numLandmarks = 6;
                if (world_mat.BUILD_GAZEBO_WORLD)
                    %world_mat.gazebo.spawnModel(world_mat.point_light_model , ...
                    %    [0, 0, .5]);
                end
                [x,y]=StateRenderer.makeCircle(3.3, numLandmarks, [.2 .2]);
                for landmarkIdx=1:numLandmarks
                    world_mat.map_landmark_colors(landmarkIdx,:) = world_mat.landmark_colors(landmarkIdx,:); 
                    world_mat.map_landmark_positions(landmarkIdx,:) = [x(landmarkIdx), ...
                        y(landmarkIdx)];
                    world_mat.makeLandmark(x(landmarkIdx), y(landmarkIdx), landmarkIdx);
                end
            elseif (map_index == 4)
                pstart=[-3.5,-3.5]';
                pend=pstart+1*world_mat.walldims(1)*dir_right;
                world_mat.makeWall(pstart, pend);
                pstart=pend;
                pend=pstart+1*world_mat.walldims(1)*dir_up;
                world_mat.makeWall(pstart, pend);
                pstart=pend;
                pend=pstart-1*world_mat.walldims(1)*dir_right;
                world_mat.makeWall(pstart, pend);
                pstart=pend;
                pend=pstart-1*world_mat.walldims(1)*dir_up;
                world_mat.makeWall(pstart, pend);
                numLandmarks = 12;
                if (world_mat.BUILD_GAZEBO_WORLD)
                    %world_mat.gazebo.spawnModel(world_mat.point_light_model , ...
                    %    [0, 0, .5]);
                end
                [x,y]=StateRenderer.makeCircle(3.3, numLandmarks, [.2 .2]);
                for landmarkIdx=1:numLandmarks
                    if (mod(landmarkIdx-1,2)==0)
                        srcIdx = mod((landmarkIdx-1)/2,6)+1;
                    else
                        srcIdx = mod(landmarkIdx/2+2,6)+1;
                    end
                    world_mat.map_landmark_colors(landmarkIdx,:) = world_mat.landmark_colors(srcIdx,:); 
                    world_mat.map_landmark_positions(landmarkIdx,:) = [x(landmarkIdx), ...
                        y(landmarkIdx)];
                    world_mat.makeLandmark(x(landmarkIdx), y(landmarkIdx), srcIdx);
                end
            end
        end
        
        function makeWall(obj, pstart, pend)
            global GUI;
            
            dir = pend - pstart;
            dir = dir./norm(dir);
            theta = atan2(dir(2),dir(1));
            R_z=[cos(theta) -sin(theta); sin(theta) cos(theta)];
            p_center=pstart+dir*3.75;
            wall_len = 0;
            polygonIndex=obj.numPolygons+1;
            while (wall_len < norm(pend-pstart))
                bb=obj.aabb;
                bb_prime = R_z*bb;
                bb_prime = bb_prime+p_center*ones(1,5);
                if (obj.BUILD_GAZEBO_WORLD)
                    obj.gazeboSpawnedModels{end+1} = obj.gazebo.spawnModel( ...
                        obj.gazebo_wall_model, ...
                        [p_center', 0],[0 0 theta]);
                end
                GUI.setFigure('MAP');
                plot(bb_prime(1,:),bb_prime(2,:));
                obj.polygonList{polygonIndex}.x=bb_prime(1,:);
                obj.polygonList{polygonIndex}.y=bb_prime(2,:);
                wall_len = wall_len + 7.5;
                p_center = p_center+dir*7.5;
                polygonIndex = polygonIndex + 1;
            end
            obj.numPolygons = length(obj.polygonList);
            axis equal;
        end
        
        function makeLandmark(obj, x, y, index)
            global GUI;
            if ~exist('index','var')
                index = 1;
            elseif (index <= 0 || index > length(obj.landmark_models))
                index = mod(index,length(obj.landmark_models));
            end
            if (obj.BUILD_GAZEBO_WORLD)
                obj.gazeboSpawnedModels{end+1} = obj.gazebo.spawnModel(...
                    obj.landmark_models{index},[x,y,1.7]);
            end
            polygonIndex=obj.numPolygons+1;
            [px, py] = StateRenderer.makeCircle(obj.landmark_diameter/2, ...
                10,[x y]);
            GUI.setFigure('MAP');
            plot( px, py, 'Color', obj.landmark_colors(index,:));
            obj.polygonList{polygonIndex}.x=px;
            obj.polygonList{polygonIndex}.y=py;
            obj.numPolygons = length(obj.polygonList);
            axis equal;
        end
        
        function bwImg = mapToBWImage(obj,width,height,im_margin)
            bwImg = zeros(height,width);
            if (exist('im_margin','var')==0)
                im_margin=[0;0];
            end
            xy_max = [-Inf;-Inf];
            xy_min = [+Inf;+Inf];
            for idx=1:obj.numPolygons
                xy_max = [max(obj.polygonList{idx}.x,xy_max(1)); ...
                    max(obj.polygonList{idx}.y,xy_max(2))];
                xy_min = [min(obj.polygonList{idx}.x,xy_min(1)); ...
                    min(obj.polygonList{idx}.y,xy_min(2))];
            end
            xy_delta=(xy_max - xy_min)./[width-2*im_margin(1); ...
                height-2*im_margin(2)];
            delta = max(xy_delta);
            for row=im_margin(2):(height-im_margin(2))
                yVal = xy_min(2) + (row-im_margin(2))*delta;
                if (mod(row-im_margin(2),0.1*(height-2*im_margin(2)))==0)
                    fprintf(1,'%0.2f percent done...\n', ...
                        100*(row-im_margin(2))/(height-2*im_margin(2)));
                end
                for col=im_margin(1):(width-im_margin(1))
                    xVal = xy_min(1) + (col-im_margin(1))*delta;
                    pt2d = [xVal; yVal];
                    for polyIdx=1:obj.numPolygons
                        if (inpolygon(pt2d(1),pt2d(2),...
                                obj.polygonList{polyIdx}.x, ...
                                obj.polygonList{polyIdx}.y))
                            bwImg(height-row,col)=1;
                        end
                    end
                end
            end
        end
        
        function consolePrint(obj, stringVal)
            if (obj.VERBOSE==1)
                disp(stringVal)
            end
        end
        
    end
    %% All GUI-related methods
    methods (Access = private)
        function pauseSimulationButtonCallback(obj, ~, ~)
            persistent state;
            %pauseSimulationButtonCallback Callback when user presses "Randomize location" button
            %pauseSimulation(obj);
            % disp('Start/Stop simulation');
            if (isempty(state)==1)
                state=true;
            end
            state = ~state;
            if (state==false)
                obj.gazebo.pauseSim();
            else
                obj.gazebo.resumeSim();
            end
        end
        
        function resetSimulationButtonCallback(obj, ~, ~)
            %resetSimulationButtonCallback Callback when user presses "Reset simulation" button
            %resetSimulation(obj);
            % disp('Reset sim');
            obj.gazebo.resetSim();
            if (isempty(obj.kobuki)==0)
                setState(obj.kobuki,'position',[0 0 0], ...
                    'orientation', [0 0 0], ...
                    'linvel',[0 0 0], 'angvel', [0 0 0]);
            end
        end
    end
end
