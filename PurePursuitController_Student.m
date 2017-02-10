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
classdef PurePursuitController_Student < OdometryListener
    %LaserScanAvoidController Avoids obstacles detected by laser scan data.
    %   Detailed explanation goes here
    
    properties
        velocityMsg
        velocityPub
        
        wayPoints
        numWayPoints
        
        goalPtIdx
        
        maxLinearVelocity
        maxAngularVelocity
        targetLinearVelocity
        lookaheadDistance
        goalRadius
        
        VISUALIZE_ALGORITHM
    end
    methods (Static)
        function rpy = quat2rpy(q)
            %QUAT2RPY Converts the quaternion in the form(q0 + q1 i + q2 j + q3 k into the roll pitch yaw
            %pitch yaw (ZYX convention) other conventions can be supported in later
            %versions. q is nx4 matrix output in radians
            if size(q,2) ~= 4
                q = q';
            end
            rpy(:,1) = atan2(2*(q(:,1).*q(:,2) +q(:,3).*q(:,4)), 1 - 2*(q(:,2).^2 + q(:,3).^2));
            rpy(:,2) = asin(2*(q(:,1).*q(:,3) -q(:,4).*q(:,2)));
            rpy(:,3) = atan2(2*(q(:,1).*q(:,4) + q(:,2).*q(:,3)), 1 - 2*(q(:,3).^2 + q(:,4).^2));
        end
    end
    
    methods
        function obj = PurePursuitController_Student(stateProvider)
            %obj@LaserScanListener();
            obj@OdometryListener(stateProvider);
            obj.velocityPub = rospublisher('/mobile_base/commands/velocity');
            obj.velocityMsg = rosmessage(obj.velocityPub);
            obj.goalRadius = 0.1;           % m
            obj.maxAngularVelocity = pi/9;  % rad/sec
            obj.maxLinearVelocity = 1.3;    % m/sec
            obj.targetLinearVelocity = 0.1; % m/sec
            obj.lookaheadDistance = 0.8;
            obj.numWayPoints = 0;
            obj.goalPtIdx = 0;
            obj.VISUALIZE_ALGORITHM = true;
        end
        
        function odometryCallback(obj, varargin)
            global START_TIME
            
            if (isa(varargin{1},'timer')==1)
                message = varargin{3}.LatestMessage;
                tfmgr = varargin{4};
            else
                message = varargin{1}.LatestMessage;
                tfmgr = varargin{2};
            end
            if (tfmgr.tftree.canTransform('map', 'base_link'))
                tfmgr.tftree.waitForTransform('map', 'base_link');
                map2basetf = tfmgr.tftree.getTransform('map', 'base_link');
                tVal = map2basetf.Transform.Translation;
                qVal = map2basetf.Transform.Rotation;
                pose = LocalPose([tVal.X tVal.Y tVal.Z], ...
                    [qVal.W qVal.X qVal.Y qVal.Z]);
                duration = rostime('now')-START_TIME;
                duration_secs = duration.Sec+duration.Nsec*10^-9;
                if (duration_secs > 10)
                    obj.doControl( pose);
                end
            else
                disp('Could not get map->base_link transform');
            end
        end
        
        function setWaypoints(obj, waypoints)
            obj.numWayPoints = size(waypoints,1);
            if (obj.numWayPoints > 0)
                obj.wayPoints = waypoints;
                obj.goalPtIdx = 1;
            else
                obj.goalPtIdx = 0;
            end
        end
        
        function doControl(obj, pose)
            currentPos = pose.position(1:2)';
            rpy=PurePursuitController.quat2rpy(pose.qorientation);
            yawAngle = rpy(3);
            if (norm(obj.wayPoints(obj.goalPtIdx,:)'-currentPos) < obj.goalRadius)
                if (obj.goalPtIdx < obj.numWayPoints)
                    obj.goalPtIdx = obj.goalPtIdx + 1;
                else
                    obj.goalPtIdx = 0;
                end
            end
            if (obj.goalPtIdx==0)
                return;
            end
            
            goalPt = obj.findGoalPoint(currentPos);
            
            %%%%% ADD CODE HERE %%%%%%%%%%

            % Command robot action
            %obj.velocityMsg.Angular.Z = w;         % optimal control
            %obj.velocityMsg.Angular.Z = 0.4*alpha; % proportional control
            obj.velocityMsg.Linear.X = obj.targetLinearVelocity;
            
            % Publish command to the motor
            send(obj.velocityPub, obj.velocityMsg);
        end
        
        function goalPt = findGoalPoint(obj, queryPt)
            persistent tris_startend tri_goal;
            
            endPt = obj.wayPoints(obj.goalPtIdx,:)';
            if (obj.goalPtIdx-1 >= 1)
                startPt = obj.wayPoints(obj.goalPtIdx-1,:)';
            else
                startPt = endPt;
            end
            
            [closestPt, goalPt] = obj.getClosestAndGoalPointsOnSegment( startPt, endPt, queryPt);
            
            if (obj.VISUALIZE_ALGORITHM==true)
                if (isempty(tris_startend)==0)
                    % remove previous scan plot
                    delete(tris_startend);
                    delete(tri_goal);
                end
                figure(1);
                polyPts=[startPt queryPt closestPt startPt queryPt endPt closestPt];
                tris_startend = plot(polyPts(1,:),polyPts(2,:),'b-');
                polyPts=[closestPt queryPt goalPt];
                tri_goal = plot(polyPts(1,:),polyPts(2,:),'r-');
            end
        end
        
        function [closestPt, goalPt] = getClosestAndGoalPointsOnSegment(obj, startPt, endPt, queryPt)
            %%%%% ADD CODE HERE %%%%%%%%%%
            closestPt = [1 1]';
            goalPt = [2 2]';
        end
    end
end

