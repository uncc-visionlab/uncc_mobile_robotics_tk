classdef RGBLandmarkEstimatorAdvanced < RGBLandmarkEstimator
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    properties
        classInfo
        numClasses
        NLOG_PROB_THRESHOLD
        MIN_BALL_SIZE_PIXELS
        %UNASSIGNED_COST
        %STATE_EVOLUTION_MODEL
        gaussFilter
    end
    
    methods
        function obj = RGBLandmarkEstimatorAdvanced(namespace)
            obj@RGBLandmarkEstimator(namespace);
            obj.VERBOSE = true;
            % 1 = RED, 2 = GREEN, 3 = BLUE, 4 = YELLOW, 5 = ORANGE
            obj.classInfo = load('rgb_landmark_classifier/class_training_data.mat');            
            %% APPLY THE CLASSIFIERS
            obj.numClasses = 5;
            landmarkColors = zeros(obj.numClasses,3);
            for classIdx=1:obj.numClasses
                landmarkColors(classIdx,:) = obj.classInfo(1).classInfo{classIdx}.Gaussian.mean./255;
            end
            obj.setLandmarkColors(landmarkColors);
            % Detection Negative Log Probability threshold for pixel classified as part of object
            % detected_pixel_class_c = -log(p(x|class=c)) < NLOG_PROB_THRESHOLD)
            obj.NLOG_PROB_THRESHOLD = 13;
            % Detection Region size threshold for pixel region classified as object
            % detected_object_class_c = region.area_pixels > MIN_BALL_SIZE_PIXELS
            obj.MIN_BALL_SIZE_PIXELS = 225;
            % Maximum allowable error between a track and a detection
            % Used to assign detected object to existing object tracks
            %obj.UNASSIGNED_COST = 99;
            %STATE_EVOLUTION_MODEL = 'ConstantVelocity';
            %obj.STATE_EVOLUTION_MODEL = 'ConstantAcceleration';
            
            % Blurring window size
            winSize=7;
            obj.gaussFilter = zeros(winSize,winSize);
            for x=1:winSize
                for y=1:winSize
                    obj.gaussFilter(y,x)=mvnpdf([x;y],0.5*(winSize+1)*ones(2,1), ...
                        (0.3*winSize)^2*eye(2));
                end
            end
            obj.gaussFilter = obj.gaussFilter./sum(sum(obj.gaussFilter));
        end
        
        function processImage(obj, imgRGB, tfmgr, tstamp)
            global GUI;
            physical_diameter_m = obj.landmarkDiameter;
            RGBCameraListener.showRGBImage(imgRGB, obj.namespace);
            try
                [idxs,centers,radii, signatures] = obj.findColoredSpheres(...
                    imgRGB, obj.landmarkColors);
            catch exception
                fprintf(1,'Error: %s\n',getReport(exception));
            end
            %[idxs,centers,radii, signatures] = RGBLandmarkEstimator.findColoredSpheres( ...
            %    imgRGB, obj.landmarkColors);
            GUI.setFigure('IMAGE', obj.namespace);
            for lidx=1:length(idxs)
                idx=idxs(lidx);
                xy_center = centers(lidx,:);
                radius = radii(lidx);
                signature = signatures(lidx,:);
                [x,y]=StateRenderer.makeCircle(radius, 10, xy_center);
                hold on, plot(x,y,'Color', obj.landmarkColors(idx,:), ...
                    'LineWidth', 0.5);
                if (~isempty(obj.Pmatrix))
                    focal_lengths = [obj.Pmatrix(1,1), obj.Pmatrix(2,2)];
                    principal_pt =[obj.Pmatrix(1,3), obj.Pmatrix(2,3)];
                    depth_m = physical_diameter_m*focal_lengths(1)/(2*radius);
                    x_m = (depth_m*(xy_center(1)-principal_pt(1))/focal_lengths(1));
                    phi = -atan2(xy_center(1)-principal_pt(1),focal_lengths(1));
                    half_subtended_angle = atan2(radius,focal_lengths(1));
                    correction =  abs((physical_diameter_m/2)*sin(half_subtended_angle));
                    radius_m = sqrt(depth_m^2+x_m^2)+correction;
                    if (obj.VERBOSE)
                        fprintf('RGBLandmarkEstimator for %s ::Landmark detected: index %d, color (%d,%d,%d),\n(Radius,Heading) (%0.2f m., %0.2f degrees)\n', ...
                            obj.namespace, idx, signature(1), signature(2), signature(3), radius_m, phi*180/pi);
                    end
                    base_link_truth = RGBCameraListener.extendTopic('/base_link_truth', obj.namespace);
                    if (tfmgr.tftree.canTransform('map', base_link_truth))
                        tfmgr.tftree.waitForTransform('map', base_link_truth);
                        map2basetf = tfmgr.tftree.getTransform('map', base_link_truth);
                        tVal = map2basetf.Transform.Translation;
                        qVal = map2basetf.Transform.Rotation;
                        pose = LocalPose([tVal.X tVal.Y tVal.Z], ...
                            [qVal.W qVal.X qVal.Y qVal.Z]);
                        rpy=PurePursuitController_Student.quat2rpy(pose.qorientation);
                        yawAngle = rpy(3);
                        %fprintf('State is (%0.2f,%0.2f,%0.2f)\n', ...
                        %    tVal.X, tVal.Y, yawAngle*180/pi);
                        landmarkPos = obj.landmarkPositions(idx,:);
                        %actual_phi = atan2(landmarkPos(2)-tVal.Y, landmarkPos(1)-tVal.X) - yawAngle;
                        actual_phi = angdiff(yawAngle, atan2(landmarkPos(2)-tVal.Y, landmarkPos(1)-tVal.X));
                        actual_radius_m = norm(landmarkPos-[tVal.X, tVal.Y]);
                        if (obj.VERBOSE)
                            %                            fprintf('RGBLandmarkEstimator for %s ::Robotactually at (x,y) (%0.2f , %0.2f)\n', ...
                            %                                obj.namespace, tVal.X, tVal.Y);
                            fprintf('RGBLandmarkEstimator for %s ::Landmark %d actually at (Radius,Heading) (%0.2f m., %0.2f degrees)\n', ...
                                obj.namespace, idx, actual_radius_m, actual_phi*180/pi);
                            fprintf('RGBLandmarkEstimator for %s ::Landmark %d position error (%0.2f m., %0.2f degrees)\n', ...
                                obj.namespace, idx, radius_m-actual_radius_m, (phi-actual_phi)*180/pi);
                        end
                        %if (~isempty(obj.landmarkPublisher))
                        if (false)
                            landmarkMsg = rosmessage(obj.landmarkPublisher);
                            landmarkMsg.Channels(1) = rosmessage('sensor_msgs/ChannelFloat32');
                            landmarkMsg.Channels(2) = rosmessage('sensor_msgs/ChannelFloat32');
                            landmarkMsg.Channels(3) = rosmessage('sensor_msgs/ChannelFloat32');
                            landmarkMsg.Channels(1).Name = 'measurement';
                            landmarkMsg.Channels(1).Values = single([actual_radius_m, actual_phi]);
                            landmarkMsg.Channels(2).Name = 'index';
                            landmarkMsg.Channels(2).Values = idx;
                            landmarkMsg.Channels(3).Name = 'signature';
                            landmarkMsg.Channels(3).Values = signature;
                            landmarkMsg.Header.Stamp = tstamp;
                            obj.landmarkPublisher.send(landmarkMsg);
                            disp('landmark published');
                        end
                    end
                    if (~isempty(obj.landmarkPublisher))
                        landmarkMsg = rosmessage(obj.landmarkPublisher);
                        landmarkMsg.Channels(1) = rosmessage('sensor_msgs/ChannelFloat32');
                        landmarkMsg.Channels(2) = rosmessage('sensor_msgs/ChannelFloat32');
                        landmarkMsg.Channels(3) = rosmessage('sensor_msgs/ChannelFloat32');
                        landmarkMsg.Channels(1).Name = 'measurement';
                        landmarkMsg.Channels(1).Values(1:2) = single([radius_m, phi]);
                        landmarkMsg.Channels(2).Name = 'index';
                        landmarkMsg.Channels(2).Values = idx;
                        landmarkMsg.Channels(3).Name = 'signature';
                        landmarkMsg.Channels(3).Values = signature;
                        landmarkMsg.Header.Stamp = tstamp;
                        obj.landmarkPublisher.send(landmarkMsg);
                        disp('landmark published');
                    end
                    pause(rand(1,1)*0.05);
                end
            end
            %obj.saveImageData(imgRGB);
        end
        
        function [idxs, centers, radii, signatures] = findColoredSpheres(obj, img_rgb, landmark_colors)
            idxs=[];
            centers=[];
            radii=[];
            signatures=[];
            [rows, cols, comps] = size(img_rgb);
            img_rgb = double(img_rgb);
            blurFrame = imfilter(img_rgb, obj.gaussFilter, 'symmetric');
            
            pixelVec = reshape(blurFrame, rows*cols, comps);
            classLogProb = zeros(rows*cols, obj.numClasses);
            % classify pixels using Gaussian distribution model
            for classIdx=1:obj.numClasses
                classLogProb(:,classIdx) = -log(mvnpdf(pixelVec, ...
                    obj.classInfo(1).classInfo{classIdx}.Gaussian.mean, ...
                    obj.classInfo(1).classInfo{classIdx}.Gaussian.cov));
            end
            [values, classIdxs] = min(classLogProb');
            classIdxs(values > obj.NLOG_PROB_THRESHOLD) = 10;
            colorIdxs = find(classIdxs < 10);
            if (length(colorIdxs) == 0)
                return;
            end
            img_vals = reshape(img_rgb(:,:,1:3),rows*cols,comps);
            labelVals = zeros(rows*cols,1);
            for pixIdx=1:length(colorIdxs)
                idx = colorIdxs(pixIdx);
                pixval = double(img_vals(idx,:))./255;
                pixval = pixval - min(pixval);
                [~, labelIdx] = max(pixval*landmark_colors');
                labelVals(idx) = labelIdx;
            end
            
            img_seg = reshape(labelVals,rows,cols);

            num_colors = size(landmark_colors,1);
            for labelVal=1:num_colors
                [y,x]=find(labelVal==img_seg);
                numpts=length(y);
                if (numpts > obj.MIN_BALL_SIZE_PIXELS)
                    xy_center = mean([x y]);
                    zero_ctr = [x y]-ones(numpts,1)*xy_center;
                    covMat = (1/(numpts-1)) * (zero_ctr'*zero_ctr);
                    eigVals = eig(covMat);
                    ecc = abs(1-eigVals(1)/eigVals(2));
                    %fprintf(1,'Detected class %d eigvals (%f,%f) ecc %f\n', ...
                    %    labelVal, eigVals(1), eigVals(2), ecc);
                    if (ecc < 0.4)
                        v = 2*sqrt(max(eigVals));
                        idxs=[idxs; labelVal];
                        centers=[centers; xy_center];
                        signatures=[signatures; landmark_colors(labelVal,:)];
                        radii=[radii; radius];
                    end
                end
            end
            
            % show segmentation result
            Lrgb = label2rgb(img_seg, 'jet', 'w', 'shuffle');
            figure(6), subplot(1,2,1), imshow(uint8(img_rgb));
            figure(6), subplot(1,2,2), imshow(Lrgb), ....
                title('Color segmentation');
            drawnow;
        end        
    end
end

