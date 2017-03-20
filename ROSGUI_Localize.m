classdef ROSGUI_Localize < ROSGUI
    %ROSGUI Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    methods (Static)
        function demo
            clear;
            clc;
            close all;
            hold off;
            global START_TIME;
            global GAZEBO_SIM;
            global GUI;
            %addpath('./bfl/pdf');
            %addpath('./bfl/model');
            %addpath('./robot_pose_ekf');
            SIMULATE = true;
            if (SIMULATE)
                WORLD_MAP_INDEX = 3;
                BUILD_GAZEBO_WORLD = true;
                ACTIVATE_KOBUKI = true;
            else
                GAZEBO_SIM = false;
                WORLD_MAP_INDEX = 0;
                BUILD_GAZEBO_WORLD = false;
                ACTIVATE_KOBUKI = false;
            end
            
            GUI = ROSGUI();
            
            if (robotics.ros.internal.Global.isNodeActive==1)
                GUI.consolePrint('Shutting down any active ROS processes....');
                rosshutdown;
                pause(3);
            end
            
            if (GUI.VERBOSE)
                GUI.consolePrint('Creating MATLAB world ....');
            end
            world_mat = WorldBuilder_MATLAB();
            GUI.world_mat = world_mat;
            
            h = GUI.getFigure('MAP');
            set(h,'Visible','on');
            h = GUI.getFigure('IMAGE');
            set(h,'Visible','on');
            %h = GUI.getFigure('ERROR');
            %set(h,'Visible','on');
            
            %ipaddress = '10.22.77.34';
            %ipaddress = '192.168.11.180';
            %ipaddress = '192.168.1.10';
            ipaddress = '10.16.30.11';
            if (robotics.ros.internal.Global.isNodeActive==0)
                GUI.consolePrint(strcat(...
                    'Initializing ROS node with master IP .... ', ...
                    ipaddress));
                % REPLACE THIS IP WITH YOUR COMPUTER / HOST IP
                % You can get your host IP by opening the "cmd" program
                % (from the "run" dialog) and typing "ipconfig" into the
                % command prompt. Review the console output and find the
                % "IPv4 Address" of your network card. These values
                % should be substituted into the ip address of the
                % command below.
                if (ispc)
                    rosinit(ipaddress,'NodeHost',GUI.ip_address);
                else
                    rosinit(ipaddress)
                end
            end
            START_TIME = rostime('now');

            if (BUILD_GAZEBO_WORLD)
                GAZEBO_SIM = true;
                world_gaz = WorldBuilder_Gazebo();
                world_mat.setGazeboBuilder(world_gaz);
                list = getSpawnedModels(world_gaz);
                if (ismember('grey_wall',list))
                    world_mat.BUILD_GAZEBO_WORLD=false;
                else
                    world_mat.BUILD_GAZEBO_WORLD=BUILD_GAZEBO_WORLD;
                end
                world_mat.makeMap(WORLD_MAP_INDEX);
                %world_gaz.removeAllTemporaryModels();
            end
            
            if (ACTIVATE_KOBUKI)
                GUI.consolePrint('Initializing a ROS TF Transform Tree....');
                world_mat.tfmgr = TFManager();
                if (false)
                    velocityPub = rospublisher('/mobile_base/commands/velocity');
                    velocityMsg = rosmessage(obj.velocityPub);
                    veloctiyMsg.Linear.X = 0.1;
                    send(velocityPub, velocityMsg);
                end
                if (GAZEBO_SIM)
                    kobuki = KobukiSim(world_gaz);
                else
                    kobuki = Kobuki();
                end
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
                elseif (WORLD_MAP_INDEX==1)
                    position = kobuki.getState();
                    world_mat.wayPoints = [position(1:2); ...
                        2 2; -2 -2; 2 -2; -2 2];
                end
                
                if (~isempty(world_mat.wayPoints))
                    path = world_mat.wayPoints
                    GUI.setFigure('MAP');
                    path_handle = plot(path(:,1), path(:,2),'k--d');
                end
                % seconds (odometry)
                %kobuki.odometryListener.setCallbackRate('fastest');
                if (SIMULATE)
                    noiseMean = zeros(6,1);
                    noiseCovariance = eye(6);
                    noiseCovariance(1:3,1:3) = 0.01*noiseCovariance(1:3,1:3);
                    noiseCovariance(4:6,4:6) = 2*(pi/180)*noiseCovariance(1:3,1:3);
                    %kobuki.odometryListener.setAddNoise(true, noiseMean, noiseCovariance);
                end
                kobuki.odometryListener.setCallbackRate(0.3, world_mat.tfmgr);
                kobuki.laserScanListener.setCallbackRate(2, world_mat.tfmgr);
                kobuki.rgbCamListener.setCallbackRate(4, world_mat.tfmgr);
                kobuki.rgbCamListener.getCameraInfo();
                %kobuki.odometryEKF.setTransformer(world_mat.tfmgr);
                
                if (isa(kobuki.velocityController,'PurePursuitController_Student'))
                    disp('Sending waypoints to pure pursuit controller.');
                    kobuki.velocityController.setWaypoints(world_mat.wayPoints);
                elseif (isa(kobuki.velocityController,'PurePursuitController'))
                    disp('Sending waypoints to pure pursuit controller.');
                    kobuki.velocityController.setWaypoints(world_mat.wayPoints);
                end
                if (~isempty(kobuki.velocityController))
                    kobuki.velocityController.setCallbackRate(0.3, world_mat.tfmgr);
                end
            end
        end
    end   
end

