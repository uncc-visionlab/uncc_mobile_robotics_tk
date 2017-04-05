classdef EKF_Localization < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        prior_mean
        prior_covariance
        
        ekfTimer
        tf_baseNode
        latestPose
        
        landmarkSubscriber
        landmarkPositions
        
        inputSubscriber
        
        loc_tform % TransformStamped
        loc_position
        loc_qorientation
        loc_stateRenderer

        pred_state
        pred_state_cov
        
        update_state
        update_state_cov
    end
    
    methods
        function obj = EKF_Localization()
            obj.prior_mean=[0 0 0]';
            obj.prior_covariance=zeros(3,3);
            obj.prior_covariance(1,1) = 4;
            obj.prior_covariance(2,2) = 1.5;
            obj.prior_covariance(2,2) = 0.5;
            obj.tf_baseNode = 'base_link_truth';

            obj.loc_tform = rosmessage('geometry_msgs/TransformStamped');
            obj.loc_tform.ChildFrameId = 'loc_base_link';
            obj.loc_tform.Header.FrameId = 'map';
           
            obj.loc_stateRenderer = StateRenderer_Gaussian(); % draws odometry state estimate
            obj.loc_stateRenderer.body_color = [0.1 0.4 0.4];
            obj.loc_stateRenderer.arrow_color = [0.1 0.6 0.6];
            
        end
        
        function setCallbackRate(obj, rate, tfmgr)
            obj.ekfTimer = timer('TimerFcn', ...
                {@obj.ekfLocalizationCallback, tfmgr}, ...
                'Period',rate,'ExecutionMode','fixedSpacing');
            pause(2);
            start(obj.ekfTimer);
        end
        
        function ekfLocalizationCallback(obj, varargin)
            persistent time_prev;

            disp('Called the ekf Localization callback');
            tfmgr = varargin{3};            
%             if (tfmgr.tftree.canTransform('map', obj.tf_baseNode))
%                 tfmgr.tftree.waitForTransform('map', obj.tf_baseNode);
%                 map2basetf = tfmgr.tftree.getTransform('map', obj.tf_baseNode);
%                 tVal = map2basetf.Transform.Translation;
%                 qVal = map2basetf.Transform.Rotation;
%                 pose = LocalPose([tVal.X tVal.Y tVal.Z], ...
%                     [qVal.W qVal.X qVal.Y qVal.Z]);
%                 obj.latestPose = pose;
%             else
%                 disp('RGBCameraListener::Could not get map->base_link transform');
%             end

            time_cur = rostime('now');
            obj.pred_state = obj.prior_mean;
            obj.pred_state_cov = obj.prior_covariance;
            obj.loc_position = [obj.pred_state(1) obj.pred_state(2) 0];
            obj.loc_qorientation = QuatLib.rpy2quat([0 0 obj.pred_state(3)]);
            if (~isempty(time_prev))
                obj.loc_tform = TFManager.populateTransformStamped( ...
                    obj.loc_tform, obj.loc_position, ...
                    obj.loc_qorientation, time_cur);                    
                %disp('OdomPropagation published odom->base_link transform to tf');
                tfmgr.tftree.sendTransform(obj.loc_tform);
                obj.loc_stateRenderer.showState(obj.loc_position, ...
                    obj.loc_qorientation);
            end
            time_prev = time_cur;            
        end

        function setControlInputTopic(obj, topic)
            obj.inputSubscriber = rossubscriber(topic, ...
                'geometry_msgs/Twist', @obj.controlInputCallback);
        end
        
        function controlInputCallback(obj, subscriber, msg)
            lin_vel = msg.Linear.X;
            ang_vel = msg.Angular.X;
            input_time = rostime('now');
            disp('EKFLocalizer heard a control signal');
            fprintf('Control signal is (Linear,Angular)  velocity (%0.2f m., %0.2f degrees)/sec\n', ...
                            lin_vel, ang_vel*180/pi);                        
        end
        
        function setLandmarkTopic(obj, topic)
            obj.landmarkSubscriber = rossubscriber(topic, ...
                'geometry_msgs/PointStamped', @obj.landmarkCallback);
        end
        
        function setLandmarkPositions(obj, landmarkPositions)
            obj.landmarkPositions = landmarkPositions;
        end
        
        function landmarkCallback(obj, subscriber, msg)
            radius_m = msg.Point.X;
            phi = msg.Point.Y;
            idx = msg.Point.Z;
            landmark_time = msg.Header.Stamp;
            disp('EKFLocalizer heard a landmark');
            fprintf('Landmark %d at (Radius,Heading) (%0.2f m., %0.2f degrees)\n', ...
                            idx, radius_m, phi*180/pi);                        
        end
        
    end
    
end

