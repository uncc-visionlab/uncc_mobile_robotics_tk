classdef ROSGUI_SLAM < ROSGUI
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
            SIMULATE = false;
            ACTIVATE_KOBUKI = true;
            BAGFILE = true;
            if (SIMULATE)
                WORLD_MAP_INDEX = 3;
                BUILD_GAZEBO_WORLD = true;
            else
                GAZEBO_SIM = false;
                WORLD_MAP_INDEX = 0;
                BUILD_GAZEBO_WORLD = false;
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
            
            if (GAZEBO_SIM==true)
                h = GUI.getFigure('ERROR');
                set(h,'Visible','on');
            end
            h = GUI.getFigure('IMAGE');
            set(h,'Visible','on');
            h = GUI.getFigure('MAP');
            set(h,'Visible','on');
            %set(h,'Color',[0.5 0.5 0.5]);
            
            %ipaddress = '127.0.0.1';
            ipaddress = '10.16.30.12';
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
            
            if (BAGFILE)
                rosparam('set', '/use_sim_time', true)
            end
            
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
                if (~BAGFILE)
                    world_mat.tfmgr.addKobuki('map','odom');
                else
                    world_mat.wayPoints = [0 0; ...
                        1.7 0; 2 -6; 1.7 0; 0 0];
                end
                
                if (isempty(world_mat.wayPoints) && 1==0)
                    pause(2);
                    uiwait(msgbox({'Specify desired robot path', ...
                        'with a sequence of waypoints using the mouse.', ...
                        'Del (remove), Double-click when done'}, 'help'));
                    world_mat.wayPoints = getline();
                    position = kobuki.getState();
                    world_mat.wayPoints = [position(1:2); ...
                        world_mat.wayPoints];
                elseif (WORLD_MAP_INDEX==1 || WORLD_MAP_INDEX == 3)
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
                %kobuki.odometryListener.setCallbackRate(0.5, world_mat.tfmgr);
                %kobuki.laserScanListener.setCallbackRate(2, world_mat.tfmgr);
                
                if (isa(kobuki.rgbCamListener,'RGBCameraListener'))
                    %kobuki.velocityController.setPoseFrame('loc_base_link');
                    if (isa(kobuki.rgbCamListener,'RGBLandmarkEstimator') || ...
                            isa(kobuki.rgbCamListener,'RGBLandmarkEstimator_Student'))
                        if (SIMULATE)
                            kobuki.rgbCamListener.setLandmarkPositions(world_mat.map_landmark_positions);
                        end
                        if (~isa(kobuki.rgbCamListener,'RGBLandmarkEstimatorAdvanced'))
                            kobuki.rgbCamListener.setLandmarkColors(world_mat.map_landmark_colors);
                            kobuki.rgbCamListener.setLandmarkDiameter(2*0.05); % 10 cm diameter markers
                        else
                            kobuki.rgbCamListener.setLandmarkDiameter(0.05); % 5 cm diameter markers
                        end
                        kobuki.rgbCamListener.setPublisher('landmarks');
                    end
                    kobuki.rgbCamListener.setCallbackRate(3, world_mat.tfmgr);
                    kobuki.rgbCamListener.getCameraInfo();
                end
                
                if (isa(kobuki.localizationEKF,'EKF_SLAM_Student') || ...
                        isa(kobuki.localizationEKF,'EKF_SLAM'))
                    %kobuki.localizationEKF.setTransformer(world_mat.tfmgr);
                    kobuki.localizationEKF.setLandmarkTopic('landmarks');
                    kobuki.localizationEKF.setControlInputTopic('/mobile_base/commands/velocity');
                    kobuki.localizationEKF.setCallbackRate(1, world_mat.tfmgr);
                    if (isa(kobuki.velocityController,'OdometryListener'))
                        %kobuki.velocityController.setOdometryTopic('ekf_loc')
                        %kobuki.velocityController.maxLinearVelocity = 0.1;
                    end
                    if (isa(kobuki.velocityController,'PurePursuitController_Student') || ...
                            isa(kobuki.velocityController,'PurePursuitController'))
                        kobuki.velocityController.setPoseFrame('/loc_base_link');
                    end
                    if (isa(kobuki.rgbCamListener,'RGBCameraListener'))
                        %kobuki.velocityController.setPoseFrame('loc_base_link');
                    end
                end
                
                
                if (isa(kobuki.velocityController,'PurePursuitController_Student') || ...
                        isa(kobuki.velocityController,'PurePursuitController'))
                    disp('Sending waypoints to pure pursuit controller.');
                    kobuki.velocityController.setWaypoints(world_mat.wayPoints);
                end
                
                if (~isempty(kobuki.velocityController))
                    kobuki.velocityController.setCallbackRate(.1, world_mat.tfmgr);
                end
            end
        end
    end
end

