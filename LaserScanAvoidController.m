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
classdef LaserScanAvoidController < LaserScanListener
    %LaserScanAvoidController Avoids obstacles detected by laser scan data.
    %   Detailed explanation goes here
    
    properties
        velocityMsg
        velocityPub
    end
    
    methods
        function obj = LaserScanAvoidController(namespace)
            obj@LaserScanListener(namespace);
            obj.velocityPub = LaserScanListener.extendTopic('/mobile_base/commands/velocity', namespace);
            obj.velocityMsg = rosmessage(obj.velocityPub);
        end
        
        function laserScanCallback(obj, varargin)
            global START_TIME
            
            if (isa(varargin{1},'timer')==1)
                laserScanMessage = varargin{3}.LatestMessage;
                tfmgr = varargin{4};
            else
                laserScanMessage = varargin{1}.LatestMessage;
                tfmgr = varargin{2};
            end
            if (tfmgr.tftree.canTransform('map', obj.tf_baseNode))
                %tfmgr.tftree.waitForTransform('map', 'base_link');
                %map2basetf = tfmgr.tftree.getTransform('map', 'base_link');
                %tVal = map2basetf.Transform.Translation;
                %qVal = map2basetf.Transform.Rotation;
                %pose = LocalPose([tVal.X tVal.Y tVal.Z], ...
                %    [qVal.W qVal.X qVal.Y qVal.Z]);
                duration = rostime('now')-START_TIME;
                duration_secs = duration.Sec+duration.Nsec*10^-9;
                if (duration_secs > 10)
                    obj.doControl(laserScanMessage);
                end
            else
                disp('LaserScanListener could not get map->base_link transform');
            end
        end
        
        function doControl(obj, laserScanMessage)
            % Define constants for TurtleBot movement
            spinVelocity = 0.30;    % Angular velocity (rad/s)
            forwardVelocity = 0.25; % Linear velocity (m/s)
            backwardVelocity = -0.05;  % Linear velocity (reverse) (m/s)
            distanceThreshold = 1.5;  % Distance threshold (m) for turning
            mindist = min(laserScanMessage.Ranges);
            % Command robot action
            if mindist < distanceThreshold
                % If close to obstacle, back up slightly and spin
                obj.velocityMsg.Angular.Z = spinVelocity;
                obj.velocityMsg.Linear.X = backwardVelocity;
            else
                % Continue on forward path
                obj.velocityMsg.Linear.X = forwardVelocity;
                obj.velocityMsg.Angular.Z = 0;
            end
            
            % Publish command to the motor
            send(obj.velocityPub,obj.velocityMsg);
        end
    end    
end

