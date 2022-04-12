classdef EKF_SLAM < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        namespace
        
        prior_mean
        prior_covariance
        
        ekfTimer
        tfPoseFrame
        latestPose
        
        landmarkSubscriber
        NEW_LANDMARK_THRESHOLD
        numLandmarks
        landmarkSignatures
        landmarkRenderers

        robotStateDim        % dimension of the robot state
        landmarkDim          % dimension of a landmark
        
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
        
        %M_t = 1e-1*[0.25 0; 0 0.25];
        M_t = [0.25 0; 0 0.25];
        Q_t = [0.2 0; 0 0.8*pi/180];
        signatureCov = [0.001 0 0; 0 0.001 0; 0 0 0.001];
        
        time_prev_controlInputCallback
        time_prev_ekfSLAMCallback
    end
    
    methods
        function obj = EKF_SLAM(namespace)
            if (exist('namespace','var'))
                obj.namespace = namespace;
            else
                obj.namespace = [];
            end
            obj.prior_mean=[0 0 0]';
            obj.prior_covariance=zeros(3,3);
            obj.prior_covariance(1,1) = 1e-5;
            obj.prior_covariance(2,2) = 1e-5;
            obj.prior_covariance(3,3) = 1e-5;

            obj.pred_state = obj.prior_mean;
            obj.pred_state_cov = obj.prior_covariance;
            
            obj.loc_tform = rosmessage('geometry_msgs/TransformStamped');
            loc_tform_ChildFrameId = OdometryListener.extendTopic('/loc_base_link', namespace);
            obj.loc_tform.ChildFrameId = loc_tform_ChildFrameId;
            %obj.loc_tform.ChildFrameId = 'loc_base_link';
            obj.loc_tform.Header.FrameId = 'map';
           
            localization_topic = OdometryListener.extendTopic('/ekf_loc', namespace);
            obj.localizationPublisher = rospublisher(localization_topic, ...
                'nav_msgs/Odometry');            
            obj.localization_odomMsg = rosmessage(obj.localizationPublisher);
            obj.localization_odomMsg.ChildFrameId = obj.loc_tform.ChildFrameId;
            obj.localization_odomMsg.Header.FrameId = 'map';
                       
            obj.loc_stateRenderer = StateRenderer_Gaussian(); % draws odometry state estimate
            obj.loc_stateRenderer.body_color = [0.1 0.4 0.4];
            obj.loc_stateRenderer.arrow_color = [0.1 0.6 0.6];
            
            obj.VERBOSE = true;
            obj.numLandmarks = 0;
            
            obj.robotStateDim = 3; % [x, y, theta], i.e., 3-dimensional robot state
            obj.landmarkDim = 2; % [x, y], i.e., 2-dimensional landmarks
            obj.NEW_LANDMARK_THRESHOLD = 15; % threshold log-cost
        end
        
        function setRobotPose(obj, position, qorientation)
            if (obj.robotStateDim==3)
                obj.pred_state(1:2) = position(1:2);                
                rpy=QuatLib.quat2rpy(qorientation);
                yawAngle = rpy(3);
                obj.pred_state(3) = yawAngle;
            elseif (obj.robotStateDim==6)
                obj.pred_state(1:3) = position;
                obj.pred_state(4:6) = qorientation(1:3);
            end
        end
        
        function setRobotPoseCovariance(obj, cov)
            obj.pred_state_cov(1:obj.robotStateDim,1:obj.robotStateDim) = cov;
        end
        
        
        function setCallbackRate(obj, rate, tfmgr)
            obj.ekfTimer = timer('TimerFcn', ...
                {@obj.ekfSLAMCallback, tfmgr}, ...
                'Period',rate,'ExecutionMode','fixedSpacing');
            obj.ekfTimer.BusyMode = 'queue';
            pause(2);
            start(obj.ekfTimer);
        end
        
        function ekfSLAMCallback(obj, varargin)
            tfmgr = varargin{3};            
            %fprintf(1,'EKF Prediction\n');
            time_cur = rostime('now');
            obj.loc_position = [obj.pred_state(1) obj.pred_state(2) 0];
            obj.loc_qorientation = QuatLib.rpy2quat([0 0 obj.pred_state(3)]);
            if (~isempty(obj.time_prev_ekfSLAMCallback))
                obj.loc_tform = TFManager.populateTransformStamped( ...
                    obj.loc_tform, obj.loc_position, ...
                    obj.loc_qorientation, time_cur);                    
                fprintf(1,'EKFLocalization::published %s->%s transform to tf.\n', obj.loc_tform.Header.FrameId, ...
                    obj.loc_tform.ChildFrameId);
                tfmgr.tftree.sendTransform(obj.loc_tform);
                obj.localization_odomMsg.Header.Stamp = time_cur;
                obj.localization_odomMsg.Pose.Pose.Position.X = obj.loc_position(1);
                obj.localization_odomMsg.Pose.Pose.Position.Y = obj.loc_position(2);
                obj.localization_odomMsg.Pose.Pose.Position.Z = obj.loc_position(3);
                obj.localization_odomMsg.Pose.Pose.Orientation.W = obj.loc_qorientation(1);
                obj.localization_odomMsg.Pose.Pose.Orientation.X = obj.loc_qorientation(2);
                obj.localization_odomMsg.Pose.Pose.Orientation.Y = obj.loc_qorientation(3);
                obj.localization_odomMsg.Pose.Pose.Orientation.Z = obj.loc_qorientation(4);                
                %obj.localizationPublisher.send(obj.localization_odomMsg);                
                obj.loc_stateRenderer.showState(obj.loc_position, ...
                    obj.loc_qorientation, obj.pred_state_cov(1:2,1:2));
                %obj.pred_state
                for lidx=1:obj.numLandmarks
                    landmarkIdx = (lidx-1)*obj.landmarkDim + obj.robotStateDim + 1;
                    landmarkState = obj.pred_state(landmarkIdx:(landmarkIdx+obj.landmarkDim-1));
                    landmarkPos = [landmarkState' 0];
                    landmarkQuat = [1 0 0 0];
                    landmarkCov = obj.pred_state_cov(landmarkIdx:(landmarkIdx+obj.landmarkDim-1),landmarkIdx:(landmarkIdx+obj.landmarkDim-1));
                    obj.landmarkRenderers{lidx}.showState(landmarkPos, ...
                        landmarkQuat, landmarkCov);
                end
            end
            obj.time_prev_ekfSLAMCallback = time_cur;            
        end

        function setControlInputTopic(obj, topic)
            obj.inputSubscriber = rossubscriber(topic, ...
                'geometry_msgs/Twist', @obj.controlInputCallback);
        end
        
        function controlInputCallback(obj, subscriber, msg)
            if (isempty(obj.time_prev_controlInputCallback))
                obj.time_prev_controlInputCallback = rostime('now');                
            end
            lin_vel = msg.Linear.X;
            ang_vel = msg.Angular.Z;
            time_cur = rostime('now');
            duration = time_cur-obj.time_prev_controlInputCallback;
            deltaT = duration.Sec+duration.Nsec*10^-9;
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

            if w_t < 0.005
%            if true
                SIMPLE_MOTION_MODEL = true;
            else
                SIMPLE_MOTION_MODEL = false;
            end
            
            if (SIMPLE_MOTION_MODEL)
                motion = deltaT*[v_t*c_thpw; v_t*s_thpw; w_t];
                % Jacobian of state (motion model)
                G_robot_t = [0 0 -deltaT*v_t*s_thpw; ...
                    0 0 deltaT*v_t*c_thpw; ...
                    0 0 0];
                % convert motion control noise into state-space noise
                V_t = [deltaT*c_thpw -deltaT^2*v_t*s_thpw; ...
                    deltaT*s_thpw deltaT^2*v_t*c_thpw; ...
                    0 deltaT];
            else
                radius = v_t/w_t;
                motion = [(-radius*s_th + radius*s_thpw); ...
                    (radius*c_th - radius*c_thpw); ...
                    w_t*deltaT];
                % Jacobian of state (motion model)
                G_robot_t = [0 0 (radius*c_th - radius*c_thpw); ...
                    0 0 (radius*s_th - radius*s_thpw); ...
                    0 0 0];
                % convert motion control noise into state-space noise
                V_t = [(-s_th + s_thpw)/w_t ((radius*(s_th-s_thpw))/w_t + radius*deltaT*c_thpw); ...
                    (c_th - c_thpw)/w_t ((-radius*(c_th-c_thpw))/w_t + radius*deltaT*s_thpw); ...
                    0 deltaT];
            end
            motion_covariance = V_t*obj.M_t*V_t';
            Imat = eye(obj.robotStateDim + obj.numLandmarks*obj.landmarkDim);
            F_x = [ eye(obj.robotStateDim) zeros(obj.robotStateDim, obj.numLandmarks*obj.landmarkDim)];
            G_t = Imat + F_x'*G_robot_t*F_x;

            % predicted state mean and covariance values
            obj.pred_state = obj.pred_state + F_x'*motion;
            obj.pred_state_cov = G_t*obj.pred_state_cov*G_t' + F_x'*motion_covariance*F_x;

            % motion = deltaT*[v_t*c_thpw; v_t*s_thpw; w_t];
            % ADD EKF PREDICTION CODE HERE TO UPDATE THE STATE (obj.pred_state)
            % 
            % obj.pred_state = obj.pred_state + motion;
            
            obj.time_prev_controlInputCallback = time_cur;
            if (obj.VERBOSE)
                fprintf('EKF_SLAM::Velocity control received (Linear,Angular)=(%0.2f m., %0.2f degrees)/sec\n', ...
                    lin_vel, ang_vel*180/pi);
                fprintf('EKF_SLAM::Duration is %0.2f sec.\n', ...
                    deltaT);
                fprintf('EKF_SLAM::Motion is (x,y,theta)=(%0.2f m., %0.2f m., %0.2f degrees)\n', ...
                    motion(1), motion(2), motion(3)*180/pi);
            end
        end
        
        function setLandmarkTopic(obj, topic)
            obj.landmarkSubscriber = rossubscriber(topic, ...
                'sensor_msgs/PointCloud', @obj.landmarkCallback);
        end
          
        function landmarkCallback(obj, subscriber, msg)
            %msg.Channels(1).Name = 'measurement';
            radius_m = msg.Channels(1).Values(1);
            phi = msg.Channels(1).Values(2);
            %msg.Channels(2).Name = 'index';
            %idx = msg.Channels(2).Values(1);
            %msg.Channels(3).Name = 'signature';
            signature = msg.Channels(3).Values;          
            landmark_time = msg.Header.Stamp;
            time_cur = rostime('now');
            duration = time_cur-landmark_time;
            deltaT = duration.Sec+duration.Nsec*10^-9;
            if (deltaT > 0.5)
                fprintf(1,'EKF_SLAM:Skipping landmark message from %0.2f secs in the past\n', deltaT);
                return;
            end
            
            z_meas = [radius_m, phi]';
            % Step 1:
            % Estimate the global coordinate system position for landmark
            % m_xy
            %     - calculate mean of the landmark position            
            %pred_xy = obj.pred_state(1:2);
            pred_theta = obj.pred_state(3);

            c_phi_p_theta = cos(phi + pred_theta);
            s_phi_p_theta = sin(phi + pred_theta);
            m_xy = obj.pred_state(1:2) + ...
                radius_m*[c_phi_p_theta; s_phi_p_theta];            

            if (obj.VERBOSE)
               fprintf('EKF_SLAM::Landmark sensed: color (%d,%d,%d), (x,y) (%0.2f, %0.2f)m.\n', ...
                            signature(1), signature(2), signature(3), m_xy(1), m_xy(2));
            end
 
            % Step 2:
            % Estimate the correspondence between the detected landmark
            % and the existing map landmarks
            % idx or 'new landmark'
            [new_landmark, H, Psi, z_hat] = estimateCorrespondence(obj, z_meas, signature);
            
            % Step 3:
            if (new_landmark)
                % If 'new landmark' add the landmark to the state
                %J_rphi_to_xy = [c_phi_p_theta -radius_m*s_phi_p_theta; ...
                %    s_phi_p_theta radius_m*c_phi_p_theta];
                %J_robot_to_xy = [eye(2) [-radius_m*s_phi_p_theta; radius_m*c_phi_p_theta]];
                J_rphi_to_xy = eye(2);
                J_robot_to_xy = [eye(2) [0;0]];
                J_crosscov = [J_robot_to_xy zeros(2,obj.numLandmarks*obj.landmarkDim)];
                for dim=1:obj.robotStateDim
                    %J_crosscov(1,(obj.robotStateDim+dim):obj.robotStateDim:end) = 1;
                end
                landmarkStateCov = J_rphi_to_xy*obj.Q_t*J_rphi_to_xy';
                robotStateCov = obj.pred_state_cov(1:obj.robotStateDim,1:obj.robotStateDim);
                landmarkStateCov = landmarkStateCov + J_robot_to_xy*robotStateCov*J_robot_to_xy';
                crossCov = J_crosscov*obj.pred_state_cov;
                %landmarkStateCov = 1e10*eye(obj.landmarkDim);
                %crossCov = 1e10*ones(2,obj.robotStateDim+obj.numLandmarks*obj.landmarkDim);

                obj.numLandmarks = obj.numLandmarks + 1;
                obj.pred_state = [obj.pred_state; m_xy];  
                obj.pred_state_cov = [obj.pred_state_cov crossCov'; ...
                    crossCov, landmarkStateCov];
                obj.landmarkSignatures(obj.numLandmarks,:) = signature';
                obj.landmarkRenderers{obj.numLandmarks} = StateRenderer_Gaussian();
                obj.landmarkRenderers{obj.numLandmarks}.body_color = [0.7 0.2 0.2];
                obj.landmarkRenderers{obj.numLandmarks}.arrow_color = [0.8 0.8 0.8];
                if (obj.VERBOSE)
                    fprintf('EKF_SLAM::Created Landmark %d at (x,y) (%0.2f, %0.2f)m. with signature (%d,%d,%d)\n', ...
                        obj.numLandmarks, m_xy(1), m_xy(2), ...
                        signature(1), signature(2), signature(3));
                end
            else
                % Else update the state mean and the covariance matrix
                % ADD EKF UPDATE CODE HERE TO UPDATE THE STATE (obj.pred_state) 
                K_t = obj.pred_state_cov*H'*inv(Psi);
                %innov_vec = [z_meas(1)-z_hat(1); ...
                %    angdiff(z_hat(2),z_meas(2))];
                innov_vec = z_meas - z_hat;
                %innov_vec
                obj.pred_state = obj.pred_state + K_t*innov_vec;
                Imat = eye(obj.robotStateDim + obj.numLandmarks*obj.landmarkDim);                
                obj.pred_state_cov =  (Imat - K_t*H)*obj.pred_state_cov;
                if (obj.VERBOSE)
                    m_xy_hat = obj.pred_state(1:2) + ...
                        z_hat(1)*[cos(z_hat(2)+pred_theta); sin(z_hat(2)+pred_theta)];
                    fprintf('EKF_SLAM::Landmark predict: color (%d,%d,%d), (x,y) (%0.2f, %0.2f)m.\n', ...
                        signature(1), signature(2), signature(3), m_xy_hat(1), m_xy_hat(2));
                end
            end            
        end
        
        function [new_landmark, H, Psi, z_hat] = estimateCorrespondence(obj, z_meas, signature)
            H=[];
            Psi=[];
            z_hat=[];
            new_landmark = true;
            pred_xy = obj.pred_state(1:2);
            pred_theta = obj.pred_state(3);
            % estimate the correspondence for a detected landmark
            % compute the log-likelihood of the observation given the
            % assumption that the observation is an existing landmark
            F_xk = [eye(obj.robotStateDim), ...
                zeros(obj.robotStateDim,obj.numLandmarks*obj.landmarkDim);
                zeros(obj.landmarkDim,obj.robotStateDim+obj.numLandmarks*obj.landmarkDim)];
            min_log_likelihood = Inf;
            log_likelihood = zeros(obj.numLandmarks,1);
            for lidx=1:obj.numLandmarks
                landmarkIdx = (lidx-1)*obj.landmarkDim + obj.robotStateDim + 1;
                landmarkState = obj.pred_state(landmarkIdx:(landmarkIdx+obj.landmarkDim-1));
                landmarkSignature = obj.landmarkSignatures(lidx,:)';
                delta_k = landmarkState - pred_xy; % vector from predicted state to landmark
                q_k = delta_k'*delta_k;
                z_hat_k = [sqrt(q_k); ...
                    angdiff(pred_theta, atan2(delta_k(2), delta_k(1)))];
                % set landmark k/lidx portion of matrix to ones
                for dim=1:obj.landmarkDim
                    F_xk(obj.robotStateDim+dim,landmarkIdx+dim-1) = 1;
                end
                sqrt_q_k = sqrt(q_k);                
                H_k = (1/q_k) * ...
                    [-sqrt_q_k*delta_k(1) -sqrt_q_k*delta_k(2) 0 ...
                     +sqrt_q_k*delta_k(1) +sqrt_q_k*delta_k(2); ...
                      delta_k(2) -delta_k(1) -q_k -delta_k(2) delta_k(1)];                  
                H_k = H_k*F_xk;
                Psi_k =  H_k*obj.pred_state_cov*H_k' + obj.Q_t;
                positionCost=(z_meas-z_hat_k)'*inv(Psi_k)*(z_meas-z_hat_k);
                signatureCost = (signature-landmarkSignature)'*inv(obj.signatureCov)*(signature-landmarkSignature);
                log_likelihood(lidx)= positionCost + signatureCost;
                % set landmark k/lidx portion of matrix to zeros
                for dim=1:obj.landmarkDim
                    F_xk(obj.robotStateDim+dim,landmarkIdx+dim-1) = 0;
                end
                if (log_likelihood(lidx) < obj.NEW_LANDMARK_THRESHOLD)
                    if (log_likelihood(lidx) < min_log_likelihood)
                        new_landmark = false;
                        min_log_likelihood = log_likelihood(lidx);
                        z_hat = z_hat_k;
                        H = H_k;
                        Psi = Psi_k;
                    end
                end
            end
            %log_likelihood
        end
    end
    
end

