classdef ROSGUI < handle
    %ROSGUI Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        VERBOSE           % show console output and debug information
        world_mat         % WorldBuilder_MATLAB object
        host_os           % Host operating system: 'Windows','MacOSX','Unix'
        ip_address
        figure_map
        figure_img
        %figure_gui
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
            
            WORLD_MAP_INDEX=1;
            BUILD_GAZEBO_WORLD=true;
            ACTIVATE_KOBUKI=true;
            
            START_TIME = -1;
            GAZEBO_SIM = false;
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
            
            if (BUILD_GAZEBO_WORLD)
                %ipaddress = '10.22.77.34';
                %ipaddress = '192.168.11.178';
                ipaddress = '10.16.30.8';
                %ipaddress = GUI.ip_address;
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
                kobuki.odometryListener.setCallbackRate(0.1, world_mat.tfmgr);
                kobuki.laserScanListener.setCallbackRate(0.5, world_mat.tfmgr);
                kobuki.rgbCamListener.setCallbackRate(4, world_mat.tfmgr);
                %kobuki.odometryEKF.setTransformer(world_mat.tfmgr);
                
                if (isa(kobuki.velocityController,'PurePursuitController_Student'))
                    disp('Sending waypoints to pure pursuit controller.');
                    kobuki.velocityController.setWaypoints(world_mat.wayPoints);
                elseif (isa(kobuki.velocityController,'PurePursuitController'))
                    disp('Sending waypoints to pure pursuit controller.');
                    kobuki.velocityController.setWaypoints(world_mat.wayPoints);
                end
                if (~isempty(kobuki.velocityController))
                    kobuki.velocityController.setCallbackRate(0.1, world_mat.tfmgr);
                end
            end
        end
    end
    
    methods
        function rosgui = ROSGUI()
            rosgui.figure_map = figure(1);
            hold on;
            set(rosgui.figure_map,'Visible','off');
            rosgui.figure_img = figure(2);
            set(rosgui.figure_img,'Visible','off');
            rosgui.VERBOSE = true;
            if (ispc)
                rosgui.host_os='Windows';
            elseif (ismac)
                rosgui.host_os='MacOSX';
            elseif (isunix)
                rosgui.host_os='Unix';
            end
            rosgui.ip_address = rosgui.getIPAddress();
            fprintf(1,'Detected host IP address: %s\n',rosgui.ip_address);
        end
        
        function setFigure(rosgui, figString)
            switch (figString)
                case 'MAP'
                    set(0,'CurrentFigure',rosgui.figure_map);
                case 'IMAGE'
                    set(0,'CurrentFigure',rosgui.figure_img);
                otherwise
                    fprintf(1,'ROSGUI::setCurrentFigure() Could not set the figure to invalid string ''%s''\n', ...
                        figString);
            end
        end
        
        function fig_h = getFigure(rosgui, figString)
            switch (figString)
                case 'MAP'
                    fig_h = rosgui.figure_map;
                case 'IMAGE'
                    fig_h = rosgui.figure_img;
                otherwise
                    fprintf(1,'ROSGUI::getFigure() Could not get the figure to invalid string ''%s''\n', ...
                        figString);
            end
        end
        
        function ipAddrString = getIPAddress(rosgui)
            switch (rosgui.host_os)
                case 'Windows'
                    [~, stdout_str] = system('ipconfig');
                case 'MacOSX'
                    [~, stdout_str] = system('ifconfig -a');
                case 'Unix'
                    [~, stdout_str] = system('ifconfig -a');
                otherwise
                    disp('No valid value for host os was found, cannot determine IP');
            end
            ip_regex = strcat('((25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)(\.)){3}', ...
                '(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)');
            ipaddresses = regexp(stdout_str, ip_regex, 'match');
            ipAddrString = '';
            for sIdx=1:length(ipaddresses)
                trimstr = strtrim(ipaddresses{sIdx});
                if (~startsWith(trimstr,'127') && ...
                        ~startsWith(trimstr,'255') && ...
                        ~endsWith(trimstr,'255'))
                    ipAddrString = trimstr;
                end
            end
        end
        
        function consolePrint(obj, stringVal)
            if (obj.VERBOSE)
                disp(stringVal)
            end
        end
    end
end

