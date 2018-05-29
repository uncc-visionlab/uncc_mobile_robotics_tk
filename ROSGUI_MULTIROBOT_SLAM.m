classdef ROSGUI_MULTIROBOT_SLAM < ROSGUI
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
            
            ipaddress = '127.0.0.1';
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
                pauseSim(world_gaz);
                phys = readPhysics(world_gaz);
                phys.UpdateRate = 50; % 100 = full speed / real time
                setPhysics(world_gaz,phys);
                resumeSim(world_gaz);
            end
            
            if (ACTIVATE_KOBUKI)
                GUI.consolePrint('Initializing a ROS TF Transform Tree....');
                world_mat.tfmgr = TFManager(2);
                
                bots(1).name = 'turtlebot1';
                bots(2).name = 'turtlebot2';
                for botIdx=1:length(bots)
                    
                    GUI.initBotFigures(bots(botIdx).name);
                    h = GUI.getFigure('IMAGE',bots(botIdx).name);
                    set(h,'Visible','on');
                    
                    kobuki = KobukiSim(world_gaz, bots(botIdx).name);
                    [bot_position, bot_euler_angles_deg] = kobuki.getState();
                    bot_qorientation = eul2quat(bot_euler_angles_deg*pi/180,'ZYX');
                    world_mat.tfmgr.addKobuki('map','odom', bots(botIdx).name, ...
                        bot_position, bot_qorientation);
                    bots(botIdx).initial_position = bot_position;
                    bots(botIdx).initial_qorientation = bot_qorientation;
                    kobuki.localizationEKF.setRobotPose(bot_position, bot_qorientation);
                    bots(botIdx).kobuki = kobuki;
                    
                    if (isempty(world_mat.wayPoints) && 1==0)
                        pause(2);
                        uiwait(msgbox({'Specify desired robot path', ...
                            'with a sequence of waypoints using the mouse.', ...
                            'Del (remove), Double-click when done'}, 'help'));
                        world_mat.wayPoints = getline();
                        pose_turtlebot = kobuki.getState();
                        world_mat.wayPoints = [pose_turtlebot(1:2); ...
                            world_mat.wayPoints];
                    elseif (WORLD_MAP_INDEX==1 || WORLD_MAP_INDEX == 3)
                        waypoints = [bot_position(1:2);...
                            -2 -2; -2 2];
                    end
                    bots(botIdx).waypoints = waypoints;
                    
                    if (~isempty(waypoints))
                        path = waypoints;
                        GUI.setFigure('MAP');
                        path_handle = plot(path(:,1), path(:,2),'k--d');
                    end
                end
                
                for botIdx=1:length(bots)
                    kobuki = bots(botIdx).kobuki;
                    
                    if (~isempty(kobuki.laserScanListener))
                        kobuki.laserScanListener.setCallbackRate(0.3, world_mat.tfmgr);
                    end

                    if (~isempty(kobuki.rgbCamListener))
                        kobuki.rgbCamListener.setCallbackRate(4, world_mat.tfmgr);
                        kobuki.rgbCamListener.getCameraInfo();
                        if (isa(kobuki.rgbCamListener,'RGBLandmarkEstimator') || ...
                                isa(kobuki.rgbCamListener,'RGBLandmarkEstimator_Student'))
                            kobuki.rgbCamListener.setLandmarkPositions(world_mat.map_landmark_positions);
                            kobuki.rgbCamListener.setLandmarkColors(world_mat.map_landmark_colors);
                            kobuki.rgbCamListener.setLandmarkDiameter(2*0.05); % 10 cm diameter markers
                            kobuki.rgbCamListener.setPublisher('landmarks');
                            %disp(world_mat.map_landmark_positions);
                        end
                    end
                    
                    if (~isempty(kobuki.localizationEKF))
                        %kobuki.localizationEKF.setTransformer(world_mat.tfmgr);
                        kobuki.localizationEKF.setCallbackRate(1, world_mat.tfmgr);
                        kobuki.localizationEKF.setLandmarkTopic(strcat(bots(botIdx).name,'/landmarks'));
                        kobuki.localizationEKF.setControlInputTopic(strcat(bots(botIdx).name,'/mobile_base/commands/velocity'));
                    end
                    
                    if (isa(kobuki.velocityController,'OdometryListener'))
                        kobuki.velocityController.setOdometryTopic(strcat(bots(botIdx).name,'/ekf_loc'))
                        kobuki.velocityController.maxLinearVelocity = 0.1;
                    end
                    
                    if (isa(kobuki.velocityController,'PurePursuitController_Student') || ...
                            isa(kobuki.velocityController,'PurePursuitController'))
                        disp('Sending waypoints to pure pursuit controller.');
                        kobuki.velocityController.setWaypoints(bots(botIdx).waypoints);
                        kobuki.velocityController.setPoseFrame('loc_base_link');
                        kobuki.velocityController.setCallbackRate(1, world_mat.tfmgr);
                    end
                end
            end
        end
    end
end

