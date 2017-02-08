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
        stateProvider
        
        odometrySub
        odometryTimer
        
        odom_position = [0 0 0];
        odom_qorientation = [1 0 0 0]; % quaternion is of the form q = [w x y z]
        odom_tform
        odom_stateRenderer
        
        initial_position = [0 0 0];
        initial_qorientation = [1 0 0 0];
        
        actual_position = [0 0 0];
        actual_qorientation = [1 0 0 0]; % quaternion is of the form q = [w x y z]
        actual_tform
        actual_stateRenderer
    end

    methods
        function obj = OdometryListener(stateProvider)
            obj.stateProvider = stateProvider;
            obj.odometrySub = rossubscriber('/odom','BufferSize',1);
            
            obj.odom_tform = rosmessage('geometry_msgs/TransformStamped');
            obj.odom_tform.ChildFrameId = 'base_link';
            obj.odom_tform.Header.FrameId = 'odom';
            obj.odom_stateRenderer = StateRenderer();

            obj.actual_tform = rosmessage('geometry_msgs/TransformStamped');
            obj.actual_tform.ChildFrameId = 'base_link_truth';
            obj.actual_tform.Header.FrameId = 'odom_truth';
            obj.actual_stateRenderer = StateRenderer();
            obj.actual_stateRenderer.body_color = [0.4 0.4 0.4];
            obj.actual_stateRenderer.arrow_color = [0.6 0.6 0.6];
        end
        
        function setCallbackRate(obj, rate, tfmgr)
            if (strcmpi(rate,'fastest')==1)
                obj.odometrySub.NewMessageFcn = ...
                    {@obj.odometryCallback, tfmgr};
            else
                obj.odometryTimer = timer('TimerFcn', ...
                    {@obj.odometryCallback,obj.odometrySub, tfmgr}, ...
                    'Period',rate,'ExecutionMode','fixedSpacing');
                pause(2);
                start(obj.odometryTimer);
            end
        end
        
        function odometryCallback(obj, varargin)
            persistent time_prev;
            
            if (isa(varargin{1},'timer')==1)
                message = varargin{3}.LatestMessage;
                tfmgr = varargin{4};
            else
                message = varargin{1}.LatestMessage;
                tfmgr = varargin{2};
            end
            time_cur = rostime(message.Header.Stamp.Sec,message.Header.Stamp.Nsec);
            odom_pos = message.Pose.Pose.Position; % .Position (point) Orientation (quat)
            obj.odom_position = [odom_pos.X odom_pos.Y odom_pos.Z];
            odom_qorient = message.Pose.Pose.Orientation;
            obj.odom_qorientation = [odom_qorient.W, odom_qorient.X, ...
                odom_qorient.Y, odom_qorient.Z];
            %pose = message.Pose.Pose; % .Position (point) Orientation (quat)
            %poseCovariance = reshape(message.Pose.Covariance,6,6);
            %twist = message.Twist.Twist; % Linear (Vector3) Angular (Vector3)
            %twistCovariance = reshape(message.Twist.Covariance,6,6);
            [position, orientation, velocity] = obj.stateProvider.getState();
            obj.actual_qorientation = eul2quat(orientation*pi/180,'ZYX');
            obj.actual_position = position;
            obj.actual_tform = obj.tfTransform(obj.actual_tform, ...
                obj.actual_position,obj.actual_qorientation);
            obj.actual_tform.Header.Stamp = rostime('now');            
            tfmgr.tftree.sendTransform(obj.actual_tform);
            obj.actual_stateRenderer.showState(obj.actual_position,obj.actual_qorientation)
            if (isempty(time_prev)==0)
                %delta_t = time_cur - time_prev;
                estimated_position = obj.initial_position + obj.odom_position;
                %estimated_qorientation = quatmultiply(obj.initial_qorientation,obj.odom_qorientation);
                rotM=quat2rotm(obj.odom_qorientation)*quat2rotm(obj.initial_qorientation);
                estimated_qorientation = rotm2quat(rotM);
                obj.odom_tform = obj.tfTransform(obj.odom_tform, ...
                    estimated_position,estimated_qorientation);
                obj.odom_tform.Header.Stamp = rostime('now');            
                %disp('OdomPropagation published odom->base_link transform to tf');
                tfmgr.tftree.sendTransform(obj.odom_tform);
                obj.odom_stateRenderer.showState(estimated_position,estimated_qorientation);
            else
                %delta_t = 0;
                % initialize actual = odom on first function call
                disp('Initializing odom position and orientation to ground truth values.');
                tfmgr.setMessage(1,'map', 'odom', obj.actual_position, ...
                    obj.actual_qorientation);
                %obj.initial_position = obj.actual_position;
                %obj.initial_qorientation = obj.actual_qorientation;
            end
            time_prev = time_cur;
        end
        
        function tform = tfTransform(~, tform, position, qorientation)
            tform.Transform.Translation.X = position(1);
            tform.Transform.Translation.Y = position(2);
            tform.Transform.Translation.Z = position(3);
            tform.Transform.Rotation.W = qorientation(1);
            tform.Transform.Rotation.X = qorientation(2);
            tform.Transform.Rotation.Y = qorientation(3);
            tform.Transform.Rotation.Z = qorientation(4);
        end
    end
end