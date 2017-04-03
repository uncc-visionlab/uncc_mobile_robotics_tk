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
    end
    
    methods
        function obj = EKF_Localization()
            obj.prior_mean=[];
            obj.prior_covariance=[];
            obj.tf_baseNode = 'base_link_truth';
        end
        
        function setCallbackRate(obj, rate, tfmgr)
            obj.ekfTimer = timer('TimerFcn', ...
                {@obj.ekfLocalizationCallback, tfmgr}, ...
                'Period',rate,'ExecutionMode','fixedSpacing');
            pause(2);
            start(obj.ekfTimer);
        end
        
        function ekfLocalizationCallback(obj, varargin)
            tfmgr = varargin{3};
            disp('Called the ekf Localization callback');
            if (tfmgr.tftree.canTransform('map', obj.tf_baseNode))
                tfmgr.tftree.waitForTransform('map', obj.tf_baseNode);
                map2basetf = tfmgr.tftree.getTransform('map', obj.tf_baseNode);
                tVal = map2basetf.Transform.Translation;
                qVal = map2basetf.Transform.Rotation;
                pose = LocalPose([tVal.X tVal.Y tVal.Z], ...
                    [qVal.W qVal.X qVal.Y qVal.Z]);
                obj.latestPose = pose;
            else
                disp('RGBCameraListener::Could not get map->base_link transform');
            end
        end
        
        function setLandmarkTopic(obj, topic)
            obj.landmarkSubscriber = rossubscriber(topic, ...
                'geometry_msgs/PointStamped', @obj.landmarkCallback);
        end
        
        function landmarkCallback(obj, subscriber, msg)
            radius_m = msg.Point.X;
            phi = msg.Point.Y;
            idx = msg.Point.Z;
            disp('EKFLocalizer heard a landmark');
            fprintf('Landmark %d at (Radius,Heading) (%0.2f m., %0.2f degrees)\n', ...
                            idx, radius_m, phi*180/pi);                        
        end
        
    end
    
end

