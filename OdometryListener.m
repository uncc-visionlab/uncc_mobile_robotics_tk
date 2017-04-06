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
classdef OdometryListener < handle
    properties
        % provides ground truth odometry via the getState() function
        % e.g. Gazebo provides this function during simulation
        stateProvider
        hasStateProvider
        
        odometrySub
        odometryTimer
        
        odom_position = [0 0 0];
        odom_qorientation = [1 0 0 0]; % quaternion is of the form q = [w x y z]
        odom_tform
        odom_stateRenderer
        
        actual_position = [0 0 0];
        actual_qorientation = [1 0 0 0]; % quaternion is of the form q = [w x y z]
        actual_tform
        actual_stateRenderer
        
        ADD_NOISE
        noiseMean
        noiseCovariance
        
        failedMsgCount
        MAX_BAD_MSGS
    end

    methods
        function obj = OdometryListener(stateProvider)
            if (isa(stateProvider,'KobukiSim'))
                obj.stateProvider = stateProvider;
                obj.hasStateProvider = true;
            else
                obj.hasStateProvider = false;
            end
            obj.ADD_NOISE = false;
            obj.odometrySub = rossubscriber('/odom','BufferSize',1);
            
            obj.odom_tform = rosmessage('geometry_msgs/TransformStamped');
            obj.odom_tform.ChildFrameId = 'base_link';
            obj.odom_tform.Header.FrameId = 'odom';

            obj.odom_stateRenderer = StateRenderer(); % draws odometry state estimate

            obj.actual_tform = rosmessage('geometry_msgs/TransformStamped');
            obj.actual_tform.ChildFrameId = 'base_link_truth';
            obj.actual_tform.Header.FrameId = 'odom_truth';

            obj.actual_stateRenderer = StateRenderer(); % draws ground truth state
            obj.actual_stateRenderer.body_color = [0.4 0.4 0.4];
            obj.actual_stateRenderer.arrow_color = [0.6 0.6 0.6];
            
            obj.failedMsgCount = 0;
            obj.MAX_BAD_MSGS = 10;            
        end
        
        function setCallbackRate(obj, rate, tfmgr)
            if (strcmpi(rate,'fastest')==1)
                obj.odometrySub.NewMessageFcn = ...
                    {@obj.odometryCallback, tfmgr};
            else
                obj.odometryTimer = timer('TimerFcn', ...
                    {@obj.odometryCallback, obj.odometrySub, tfmgr}, ...
                    'Period',rate,'ExecutionMode','fixedSpacing');
                pause(2);
                start(obj.odometryTimer);
            end
        end
        
        function setAddNoise(obj, value, mean, covariance)
            obj.ADD_NOISE = value;
            if (exist('mean','var'))
                obj.noiseMean = mean;
            end
            if (exist('covariance','var'))
                obj.noiseCovariance = covariance;
            end
        end
        
        function odometryCallback(obj, varargin)
            global GAZEBO_SIM;
            persistent time_prev;
            
            if (isa(varargin{1},'timer')==1)
                message = varargin{3}.LatestMessage;
                tfmgr = varargin{4};
            else
                message = varargin{1}.LatestMessage;
                tfmgr = varargin{2};
            end
            
            if (isempty(message))
                disp('OdometryListener::Skipping empty odometry message.');
                obj.failedMsgCount = obj.failedMsgCount + 1;
                if (obj.failedMsgCount > obj.MAX_BAD_MSGS)
                    delete obj;
                else
                    return;
                end
            else
                obj.failedMsgCount = 0;
            end
            
            %time_cur = rostime(message.Header.Stamp.Sec,message.Header.Stamp.Nsec);
            time_cur = message.Header.Stamp;
            
            odom_pos = message.Pose.Pose.Position; % .Position (point) Orientation (quat)
            obj.odom_position = [odom_pos.X odom_pos.Y odom_pos.Z];
            odom_qorient = message.Pose.Pose.Orientation;
            obj.odom_qorientation = [odom_qorient.W, odom_qorient.X, ...
                odom_qorient.Y, odom_qorient.Z];
            if (obj.ADD_NOISE) % add noise
                mvn_sample = mvnrnd(obj.noiseMean,obj.noiseCovariance);
                position_noise = mvn_sample(1:3);
                qorientation_noise = eul2quat(mvn_sample(4:6)*pi/180,'ZYX');
                obj.odom_position = obj.odom_position + position_noise;
                obj.odom_qorientation = obj.odom_qorientation + qorientation_noise;
            end
            %pose = message.Pose.Pose; % .Position (point) Orientation (quat)
            %poseCovariance = reshape(message.Pose.Covariance,6,6);
            %twist = message.Twist.Twist; % Linear (Vector3) Angular (Vector3)
            %twistCovariance = reshape(message.Twist.Covariance,6,6);
            if (obj.hasStateProvider)
                [position, orientation, velocity] = obj.stateProvider.getState();
                obj.actual_qorientation = eul2quat(orientation*pi/180,'ZYX');
                %obj.actual_qorientation = QuatLib.rpy2quat(orientation*pi/180);
                obj.actual_position = position;
                obj.actual_tform = TFManager.populateTransformStamped( ...
                    obj.actual_tform, ...
                    obj.actual_position, obj.actual_qorientation, ...
                    rostime('now'));                    
                tfmgr.tftree.sendTransform(obj.actual_tform);
                obj.actual_stateRenderer.showState(obj.actual_position, ...
                    obj.actual_qorientation)
            end
            
            if (isempty(time_prev)==0)
                %delta_t = time_cur - time_prev;
                obj.odom_tform = TFManager.populateTransformStamped( ...
                    obj.odom_tform, obj.odom_position, ...
                    obj.odom_qorientation, message.Header.Stamp);                    
                %disp('OdomPropagation published odom->base_link transform to tf');
                tfmgr.tftree.sendTransform(obj.odom_tform);
                obj.odom_stateRenderer.showState(obj.odom_position, ...
                    obj.odom_qorientation);
            elseif (obj.hasStateProvider)
                %delta_t = 0;
                % initialize actual = odom on first function call
                disp('Initializing odom position and orientation to ground truth values.');
                tfmgr.setMessage(1,'map', 'odom', obj.actual_position, ...
                    obj.actual_qorientation);
            end
            time_prev = time_cur;
        end
    end
end