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
classdef PIDController < OdometryPathRecorder
    
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
        goalRadius

        tfPoseFrame
        
        VISUALIZE_ALGORITHM
        VISUALIZE_METRICS
        
        actual_path
        path_error
        
        tris_startend
        tri_goal
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
        function obj = PIDController(stateProvider, namespace)
            obj@OdometryPathRecorder(stateProvider, namespace);
            topic_vel = OdometryListener.extendTopic('/mobile_base/commands/velocity', namespace);
            obj.velocityPub = rospublisher(topic_vel,'geometry_msgs/Twist');
            obj.velocityMsg = rosmessage(obj.velocityPub);
            obj.closest_pathPts = zeros(obj.MAX_VALUES,3);
            obj.goalRadius = 0.1;           % m
            obj.maxAngularVelocity = pi/4;  % rad/sec
            obj.maxLinearVelocity = 0.15;    % m/sec
            obj.numWayPoints = 0;
            obj.goalPtIdx = 0;
            obj.VISUALIZE_ALGORITHM = false;
            obj.VISUALIZE_METRICS = true;
            obj.tfPoseFrame = OdometryListener.extendTopic('/base_link', namespace); % use odometry for pose
        end
        
        function odometryCallback(obj, varargin)
            global START_TIME GUI;
            %persistent actual_path path_error;
            
            if (isa(varargin{1},'timer')==1)
                tfmgr = varargin{4};
            else
                tfmgr = varargin{2};
            end
            obj.odometryCallback@OdometryPathRecorder(varargin{:});            
            if (tfmgr.tftree.canTransform('map', obj.tfPoseFrame))
                tfmgr.tftree.waitForTransform('map', obj.tfPoseFrame);
                map2basetf = tfmgr.tftree.getTransform('map', obj.tfPoseFrame);
                tVal = map2basetf.Transform.Translation;
                qVal = map2basetf.Transform.Rotation;
                pose = LocalPose([tVal.X tVal.Y tVal.Z], ...
                    [qVal.W qVal.X qVal.Y qVal.Z]);
                duration = rostime('now')-START_TIME;
                duration_secs = duration.Sec+duration.Nsec*10^-9;
                if (duration_secs > 10)
                    obj.doControl(pose);
                end
            else
                disp('PurePursuitController could not get map->base_link transform');
            end
            if (obj.VISUALIZE_METRICS)
                if (~isempty(obj.actual_path))
                    % remove previous plots
                    delete(obj.actual_path);
                    if (obj.goalPtIdx > 0 && obj.numPathPts > 1)
                        delete(obj.path_error);
                    end
                end
                GUI.setFigure('MAP');
                obj.actual_path = plot(obj.odom_pathPts(1:obj.numPathPts,1), ...
                    obj.odom_pathPts(1:obj.numPathPts,2), ':', 'Color', [1 0.5 0]);
                if (obj.goalPtIdx > 0 && obj.numPathPts > 1)
                    endPt = obj.wayPoints(obj.goalPtIdx,:)';
                    if (obj.goalPtIdx-1 >= 1)
                        startPt = obj.wayPoints(obj.goalPtIdx-1,:)';
                    else
                        startPt = endPt;
                    end
                    %GUI.setFigure('ERROR');
                    error_image = 'ERROR';
                    GUI.setFigure(error_image, obj.namespace);
                    if (exist('pose','var')==true)
                        queryPt = pose.position(1:2)';
                        closestPt = PIDController.closestPointOnSegment( startPt, endPt, queryPt);
                        obj.pathError(obj.numPathPts) = norm(closestPt-queryPt);
                        totalError = trapz(obj.recorded_times(1:obj.numPathPts), ...
                            obj.pathError(1:obj.numPathPts));
                        obj.path_error = plot(obj.recorded_times(1:obj.numPathPts), ...
                            obj.pathError(1:obj.numPathPts),'r-');
                        xlabelstr = sprintf('Total error: %0.3f Elapsed time: %0.3f (secs)', ...
                            totalError, obj.recorded_times(obj.numPathPts));
                        xlabel(xlabelstr);
                    end
                end
            end            
        end
        
        function setPoseFrame(obj, frame_name)
            if (~isempty(obj.namespace))
                frame_name = strcat(obj.namespace,'/',frame_name);
            end
            obj.tfPoseFrame = frame_name;
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
            rpy=PIDController.quat2rpy(pose.qorientation);
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
            
            goalPt = obj.wayPoints(obj.goalPtIdx,:)';
            vecToGoal = goalPt - currentPos;
            angleToGoal = atan2(vecToGoal(2), vecToGoal(1));
            angleError = wrapToPi(angleToGoal - yawAngle);
            edgeLength = norm(obj.wayPoints(obj.goalPtIdx,:)-obj.wayPoints(obj.goalPtIdx-1,:));
            obj.velocityMsg.Angular.Z = 0.4*angleError;
            obj.velocityMsg.Linear.X = 0.5*obj.maxLinearVelocity+0.5*(sin(pi*norm(vecToGoal)/edgeLength));
            
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
            % fprintf(1,'Publishing velocity command to topic : %s\n',obj.velocityPub.TopicName);
            send(obj.velocityPub, obj.velocityMsg);
        end
    end
end

