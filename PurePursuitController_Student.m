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
classdef PurePursuitController_Student < OdometryPathRecorder
    %LaserScanAvoidController Avoids obstacles detected by laser scan data.
    %   Detailed explanation goes here
    
    properties
        velocityMsg
        velocityPub
        
        wayPoints
        numWayPoints
        
        closest_pathPts
        pathError
        
        goalPtIdx
        
        maxLinearVelocity
        maxAngularVelocity
        targetLinearVelocity
        lookaheadDistance
        goalRadius
        
        VISUALIZE_ALGORITHM
        VISUALIZE_METRICS
    end
    methods (Static)
        function pathPt = closestPointOnSegment(p1, p2, queryPt)
            edgeVec = (p2 - p1);
            edgeLengthSquared = edgeVec'*edgeVec;
            if (edgeLengthSquared == 0.0)
                pathPt = p1;
                return;
            end
            
            % Consider the line extending the segment, parameterized as p1 + t (p2 - p1).
            % We find projection of point p onto the line.
            % It falls where t = [(p-p1) . (p2-p1)] / |p2-p1|^2
            t = (queryPt - p1)' * edgeVec / edgeLengthSquared;
            
            pathPt = p1 + t * edgeVec;
            if (t < 0.0)
                pathPt = p1; % Beyond the 'pt1' end of the segment
            elseif (t > 1.0)
                pathPt = p2; % Beyond the 'pt2' end of the segment
            end
        end
        
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
            obj@OdometryPathRecorder(stateProvider);
            obj.closest_pathPts = zeros(obj.MAX_VALUES,3);
            obj.velocityPub = rospublisher('/mobile_base/commands/velocity');
            obj.velocityMsg = rosmessage(obj.velocityPub);
            obj.goalRadius = 0.1;           % m
            obj.maxAngularVelocity = pi/4;  % rad/sec
            obj.maxLinearVelocity = 0.3;    % m/sec
            obj.lookaheadDistance = 0.8;
            obj.numWayPoints = 0;
            obj.goalPtIdx = 0;
            obj.VISUALIZE_ALGORITHM = false;
            obj.VISUALIZE_METRICS = true;
        end
        
        function odometryCallback(obj, varargin)
            global START_TIME GUI;
            persistent actual_path path_error;
            
            if (isa(varargin{1},'timer')==1)
                %message = varargin{3}.LatestMessage;
                tfmgr = varargin{4};
            else
                %message = varargin{1}.LatestMessage;
                tfmgr = varargin{2};
            end
            obj.odometryCallback@OdometryPathRecorder(varargin{:});
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
            if (obj.VISUALIZE_METRICS)
                if (isempty(actual_path)==0)
                    % remove previous plots
                    delete(actual_path);
                    if (obj.goalPtIdx > 0 && obj.numPathPts > 1)
                        delete(path_error);
                    end
                end
                GUI.setFigure('MAP');
                actual_path = plot(obj.actual_pathPts(1:obj.numPathPts,1), ...
                    obj.actual_pathPts(1:obj.numPathPts,2), ':', 'Color', [1 0.5 0]);
                if (obj.goalPtIdx > 0 && obj.numPathPts > 1)
                    endPt = obj.wayPoints(obj.goalPtIdx,:)';
                    if (obj.goalPtIdx-1 >= 1)
                        startPt = obj.wayPoints(obj.goalPtIdx-1,:)';
                    else
                        startPt = endPt;
                    end
                    GUI.setFigure('ERROR');
                    queryPt = pose.position(1:2)';
                    closestPt = PurePursuitController_Student.closestPointOnSegment( startPt, endPt, queryPt);
                    obj.pathError(obj.numPathPts) = norm(closestPt-queryPt);
                    totalError = trapz(obj.recorded_times(1:obj.numPathPts), ...
                        obj.pathError(1:obj.numPathPts));
                    path_error = plot(obj.recorded_times(1:obj.numPathPts), ...
                        obj.pathError(1:obj.numPathPts),'r-');
                    xlabelstr = sprintf('Total error: %0.3f Elapsed time: %0.3f (secs)', ...
                        totalError, obj.recorded_times(obj.numPathPts));
                    xlabel(xlabelstr);
                end
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
            rpy=PurePursuitController_Student.quat2rpy(pose.qorientation);
            yawAngle = rpy(3);
            if (obj.goalPtIdx==0)
                return;
            end
            if (norm(obj.wayPoints(obj.goalPtIdx,:)'-currentPos) < obj.goalRadius)
                if (obj.goalPtIdx < obj.numWayPoints)
                    obj.goalPtIdx = obj.goalPtIdx + 1;
                else
                    obj.goalPtIdx = 0;
                    return;
                end
            end
            
            goalPt = obj.findGoalPoint(currentPos);

            %%%%% ADD CODE HERE %%%%%%%%%%
            
            % Command robot action
            %obj.velocityMsg.Angular.Z = w;         % optimal control
            %obj.velocityMsg.Angular.Z = 0.4*angleError; % proportional control
            obj.velocityMsg.Linear.X = obj.maxLinearVelocity;
            
            % Clamp velocity command values
            if (obj.velocityMsg.Angular.Z > obj.maxAngularVelocity)
                obj.velocityMsg.Angular.Z = obj.maxAngularVelocity;
            elseif (obj.velocityMsg.Angular.Z < -obj.maxAngularVelocity)
                obj.velocityMsg.Angular.Z = -obj.maxAngularVelocity;
            end
            if (obj.velocityMsg.Linear.X > obj.maxLinearVelocity)
                obj.velocityMsg.Linear.X = obj.maxLinearVelocity;
            elseif (obj.velocityMsg.Linear.X < -obj.maxLinearVelocity)
                obj.velocityMsg.Linear.X = -obj.maxLinearVelocity;
            end
            % Publish command to the motor
            send(obj.velocityPub, obj.velocityMsg);
        end
        
        function goalPt = findGoalPoint(obj, queryPt)
            global GUI;
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
                %figure(1);
                GUI.setFigure('MAP');
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

