classdef EKF_Localization_Student < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        namespace
        
        prior_mean
        prior_covariance
        
        ekfTimer
        latestPose
        
        landmarkSubscriber
        landmarkPositions
        
        VERBOSE
        
        inputSubscriber

        localizationPublisher
        localization_odomMsg

        loc_tform % TransformStamped
        loc_position
        loc_qorientation
        loc_stateRenderer

        pred_state
        pred_state_cov
        
        M_t = [0.25 0; 0 0.25];
        Q_t = [0.2 0 0; 0 0.8*pi/180 0; 0 0 1];
        
        time_prev_controlInputCallback
        time_prev_ekfLocalizationCallback        
    end
    
    methods
        function obj = EKF_Localization_Student(namespace)
            if (exist('namespace','var'))
                obj.namespace = namespace;
            else
                obj.namespace = [];
            end            
            obj.prior_mean=[0 0 0]';
            obj.prior_covariance=zeros(3,3);
            obj.prior_covariance(1,1) = .4;
            obj.prior_covariance(2,2) = .2;
            obj.prior_covariance(3,3) = 0.05;

            obj.pred_state = obj.prior_mean;
            obj.pred_state_cov = obj.prior_covariance;
            
            obj.loc_tform = rosmessage('geometry_msgs/TransformStamped');
            loc_tform_ChildFrameId = strcat(namespace,'/loc_base_link');            
            %obj.loc_tform.ChildFrameId = 'loc_base_link';
            obj.loc_tform.ChildFrameId = loc_tform_ChildFrameId;
            obj.loc_tform.Header.FrameId = 'map';
           
            localization_topic = strcat(namespace,'/ekf_loc');
            obj.localizationPublisher = rospublisher(localization_topic, ...
                'nav_msgs/Odometry');
            obj.localization_odomMsg = rosmessage(obj.localizationPublisher);
            
            obj.loc_stateRenderer = StateRenderer_Gaussian(); % draws odometry state estimate
            obj.loc_stateRenderer.body_color = [0.1 0.4 0.4];
            obj.loc_stateRenderer.arrow_color = [0.1 0.6 0.6];
            
            obj.VERBOSE = false;
        end
        
        function setCallbackRate(obj, rate, tfmgr)
            obj.ekfTimer = timer('TimerFcn', ...
                {@obj.ekfLocalizationCallback, tfmgr}, ...
                'Period',rate,'ExecutionMode','fixedSpacing');
            obj.ekfTimer.BusyMode = 'queue';
            pause(2);
            start(obj.ekfTimer);
        end
        
        function ekfLocalizationCallback(obj, varargin)
            tfmgr = varargin{3};            
            
            time_cur = rostime('now');
            obj.loc_position = [obj.pred_state(1) obj.pred_state(2) 0];
            obj.loc_qorientation = QuatLib.rpy2quat([0 0 obj.pred_state(3)]);
            if (~isempty(obj.time_prev_ekfLocalizationCallback))
                obj.loc_tform = TFManager.populateTransformStamped( ...
                    obj.loc_tform, obj.loc_position, ...
                    obj.loc_qorientation, time_cur);                    
                %disp('EKFLocalization::published odom->base_link transform to tf');
                tfmgr.tftree.sendTransform(obj.loc_tform);
                obj.localization_odomMsg.Header.Stamp = time_cur;
                obj.localization_odomMsg.Pose.Pose.Position.X = obj.loc_position(1);
                obj.localization_odomMsg.Pose.Pose.Position.Y = obj.loc_position(2);
                obj.localization_odomMsg.Pose.Pose.Position.Z = obj.loc_position(3);
                obj.localization_odomMsg.Pose.Pose.Orientation.W = obj.loc_qorientation(1);
                obj.localization_odomMsg.Pose.Pose.Orientation.X = obj.loc_qorientation(2);
                obj.localization_odomMsg.Pose.Pose.Orientation.Y = obj.loc_qorientation(3);
                obj.localization_odomMsg.Pose.Pose.Orientation.Z = obj.loc_qorientation(4);                
                obj.localizationPublisher.send(obj.localization_odomMsg);                
                obj.loc_stateRenderer.showState(obj.loc_position, ...
                    obj.loc_qorientation, obj.pred_state_cov(1:2,1:2));
            end
            obj.time_prev_ekfLocalizationCallback = time_cur;            
        end

        function setControlInputTopic(obj, topic)
            obj.inputSubscriber = rossubscriber(topic, ...
                'geometry_msgs/Twist', @obj.controlInputCallback);
        end
        
        function controlInputCallback(obj, subscriber, msg)
            persistent time_prev;            
            if (isempty(obj.time_prev_controlInputCallback))
                obj.time_prev_controlInputCallback = rostime('now');                
            end
            lin_vel = msg.Linear.X;
            ang_vel = msg.Angular.Z;
            time_cur = rostime('now');
            duration = time_cur - obj.time_prev_controlInputCallback;
            deltaT = duration.Sec + duration.Nsec*10^-9;
            if (deltaT <= 0)
                return;
            end
            v_t = lin_vel;
            w_t = ang_vel;
            theta = obj.pred_state(3);

            s_th = sin(theta);
            s_thpw = sin(theta+deltaT*w_t);
            c_th = cos(theta);
            c_thpw = cos(theta+deltaT*w_t);
           
            motion = deltaT*[v_t*c_thpw; v_t*s_thpw; w_t];

            % ADD EKF PREDICTION CODE HERE TO UPDATE THE STATE (obj.pred_state)
            
            obj.pred_state = obj.pred_state + motion;
            
            obj.time_prev_controlInputCallback = time_cur;
            if (obj.VERBOSE)
                fprintf('EKFLocalization for %s::Velocity control received (Linear,Angular)=(%0.2f m., %0.2f degrees)/sec\n', ...
                    obj.namespace, lin_vel, ang_vel*180/pi);
                fprintf('EKFLocalization for %s::Duration is %0.2f sec.\n', ...
                    obj.namespace, deltaT);
                fprintf('EKFLocalization for %s::Motion is (x,y,theta)=(%0.2f m., %0.2f m., %0.2f degrees)\n', ...
                    obj.namespace, motion(1), motion(2), motion(3)*180/pi);
            end
        end
        
        function setLandmarkTopic(obj, topic)
            obj.landmarkSubscriber = rossubscriber(topic, ...
                'sensor_msgs/PointCloud', @obj.landmarkCallback);
        end
        
        function setLandmarkPositions(obj, landmarkPositions)
            obj.landmarkPositions = landmarkPositions;
        end
        
        function landmarkCallback(obj, subscriber, msg)
            %msg.Channels(1).Name = 'measurement';
            radius_m = msg.Channels(1).Values(1);
            phi = msg.Channels(1).Values(2);
            %msg.Channels(2).Name = 'index';
            idx = msg.Channels(2).Values(1);
            %msg.Channels(3).Name = 'signature';
            signature = msg.Channels(3).Values;          
            %landmark_time = msg.Header.Stamp;           

            z=[radius_m; phi; 0];
            
            m_xy = obj.landmarkPositions(idx,:)';
            pred_loc_to_landmark_vec = m_xy - obj.pred_state(1:2);
            pred_theta = obj.pred_state(3);

            % ADD EKF UPDATE CODE HERE TO UPDATE THE STATE (obj.pred_state)
            
            if (obj.VERBOSE)
                fprintf('EKFLocalization for %s::Landmark %d at (Radius,Heading) (%0.2f m., %0.2f degrees)\n', ...
                    obj.namespace, idx, radius_m, phi*180/pi);
            end
        end
        
    end
    
end

