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
            world_mat = WorldBuilder_MATLAB();
            global START_TIME;
            global GAZEBO_SIM;
            
            rosshutdown;
            world_mat.consolePrint('Shutting down any active ROS processes....');
            pause(1);
            
            GAZEBO_SIM = false;
            WORLD_MAP_INDEX=1;
            world_mat.BUILD_GAZEBO_WORLD=1;
            ACTIVATE_KOBUKI=1;
                        
            if (world_mat.BUILD_GAZEBO_WORLD==1)
                %ipaddress = '10.22.77.34';
                %ipaddress = '192.168.11.180';
                ipaddress = '10.16.30.15';
                if (robotics.ros.internal.Global.isNodeActive==0)
                    world_mat.consolePrint(strcat('Initializing ROS node with master IP ....', ...
                        ipaddress));
                    rosinit(ipaddress)
                end
                START_TIME = rostime('now');
                GAZEBO_SIM = true;
                world_gaz = WorldBuilder_Gazebo();
                world_mat.setGazeboBuilder(world_gaz);
                list = getSpawnedModels(world_gaz);
                if (ismember('grey_wall',list))
                    world_mat.BUILD_GAZEBO_WORLD=0;
                end
                world_mat.consolePrint('Initializing a ROS TF Transform Tree....');
                world_mat.tfmgr = TFManager();
                %world_gaz.removeAllTemporaryModels();
            end                        
            
            dir_right=[1,0]';
            dir_up=[0,1]';
            if (WORLD_MAP_INDEX==1)
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
            elseif (WORLD_MAP_INDEX==2)
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
                %image_frame=getframe(figure(1));
                
                bwImg = world_mat.mapToBWImage(455,245,[10; 10]);
                figure(2), imshow(bwImg);
            end
            
            
            if (ACTIVATE_KOBUKI==1)
                kobuki = Kobuki(world_gaz);
                world_mat.kobuki = kobuki;
                
                if (isempty(world_mat.wayPoints) && 1==0)
                    pause(2);
                    uiwait(msgbox({'Specify desired robot path', ...
                        'with a sequence of waypoints using the mouse.', ...
                        'Del (remove), Double-click when done'}, 'help'));
                    world_mat.wayPoints = getline();
                    position = kobuki.getState();
                    world_mat.wayPoints = [position(1:2); ...
                        world_mat.wayPoints];
                else
                    position = kobuki.getState();
                    world_mat.wayPoints = [position(1:2); ...
                        2 2; -2 -2; 2 -2; -2 2];
                end
                
                if (~isempty(world_mat.wayPoints))
                    path = world_mat.wayPoints
                    figure(1), plot(path(:,1), path(:,2),'k--d');
                end
                % seconds (odometry)
                %kobuki.odometryListener.setCallbackRate('fastest');
                kobuki.odometryListener.setCallbackRate(0.1, world_mat.tfmgr);
                kobuki.laserScanListener.setCallbackRate(0.5, world_mat.tfmgr);
                kobuki.rgbCamListener.setCallbackRate(4, world_mat.tfmgr);
                % reassign the velocity controller
                %kobuki.velocityController = LaserScanAvoidController();
                kobuki.velocityController = PurePursuitController_Student(world_gaz);

                if (isa(kobuki.velocityController,'PurePursuitController_Student'))
                    disp('Sending waypoints to pure pursuit controller.');
                    kobuki.velocityController.setWaypoints(world_mat.wayPoints);
                end
                kobuki.velocityController.setCallbackRate(0.1, world_mat.tfmgr);
            end            
        end
    end
    methods
        function obj = WorldBuilder_MATLAB()
            obj.walldims=[7.5,0.2,2.8]';
            obj.aabb=[-3.75 3.75 3.75 -3.75 -3.75;
                0.1 0.1 -0.1 -0.1 0.1];
            obj.numPolygons = 0;
            obj.VERBOSE = 1;
            % Create buttons in the GUI window
            obj.randomLocationButton = uicontrol(figure(1), ...
                'Style', 'pushbutton', ...
                'String','Randomize Location', ...
                'Units', 'normalized', ...
                'Position', [0.80 0.01 0.2 0.05], ...
                'Callback', @obj.randomLocationButtonCallback);
            
            obj.resetSimulationButton = uicontrol(figure(1), ...
                'Style', 'pushbutton', ...
                'String','Reset Simulation', ...
                'Units', 'normalized', ...
                'Position', [0.0 0.01 0.2 0.05], ...
                'Callback', @obj.resetSimulationButtonCallback);
            
        end
        
        function setGazeboBuilder(obj, gazbuilder)
            obj.gazebo = gazbuilder;
            obj.gazebo_wall_model = ExampleHelperGazeboModel('grey_wall','gazeboDB');
        end
        
        function makeWall(obj, pstart, pend)
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
                if (obj.BUILD_GAZEBO_WORLD==1)
                    obj.gazebo.spawnModel(obj.gazebo_wall_model, ...
                        [p_center', 0],[0 0 theta]);
                end
                hold on, plot(bb_prime(1,:),bb_prime(2,:));
                obj.polygonList.x(polygonIndex,:)=bb_prime(1,:);
                obj.polygonList.y(polygonIndex,:)=bb_prime(2,:);
                wall_len = wall_len + 7.5;
                p_center = p_center+dir*7.5;
                polygonIndex = polygonIndex + 1;
            end
            obj.numPolygons = size(obj.polygonList.x,1);
            axis equal;
        end
        
        function bwImg = mapToBWImage(obj,width,height,im_margin)
            bwImg = zeros(height,width);
            if (exist('im_margin','var')==0)
                im_margin=[0;0];
            end
            xy_max = [max(max(obj.polygonList.x)); ...
                max(max(obj.polygonList.y))];
            xy_min = [min(min(obj.polygonList.x)); ...
                min(min(obj.polygonList.y))];
            xy_delta=(xy_max - xy_min)./[width-2*im_margin(1); ...
                height-2*im_margin(2)];
            delta = max(xy_delta);
            for row=im_margin(2):(height-im_margin(2))
                yVal = xy_min(2) + (row-im_margin(2))*delta;
                %yVal = xy_max(2) - (row-1)*delta;
                if (mod(row-im_margin(2),0.1*(height-2*im_margin(2)))==0)
                    fprintf(1,'%0.2f percent done...\n', ...
                        100*(row-im_margin(2))/(height-2*im_margin(2)));
                end
                for col=im_margin(1):(width-im_margin(1))
                    xVal = xy_min(1) + (col-im_margin(1))*delta;
                    pt2d = [xVal; yVal];
                    for polyIdx=1:obj.numPolygons
                        if (inpolygon(pt2d(1),pt2d(2),...
                                obj.polygonList.x(polyIdx,:), ...
                                obj.polygonList.y(polyIdx,:)))
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
        function randomLocationButtonCallback(obj, ~, ~)
            %randomLocationButtonCallback Callback when user presses "Randomize location" button
            %randomizeLocation(obj);
            disp('Random location');
        end
        
        function resetSimulationButtonCallback(obj, ~, ~)
            %resetSimulationButtonCallback Callback when user presses "Reset simulation" button
            %resetSimulation(obj);
            disp('Reset sim');
        end
    end    
end
