classdef RGBLandmarkEstimator_Student < RGBCameraListener
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties       
        landmarkPositions
        landmarkColors
        landmarkDiameter
        landmarkPublisher
    end
    methods (Static)
        function test
            clear;
            close all;
            load('imageData.mat');
            img_rgb = imageInfo.img;
            K = imageInfo.Kmatrix
            figure(1), imshow(img_rgb);
            landmark_colors = [1.0 0.0 0.0;
                0.0 1.0 0.0;
                0.0 0.0 1.0;
                1.0 1.0 0.0;
                0.0 1.0 1.0;
                1.0 0.0 1.0;
                0.5 0.5 0.0;
                0.0 0.5 0.5;
                0.5 0.0 0.5];            
            %vg=[1 1 1]/sqrt(3);
            %[rows,cols,comps]=size(img_rgb);
            %img_vals = reshape(double(img_rgb),rows*cols,comps);
            %img_vals = img_vals-(img_vals*vg').*img_vals;
            %imshow(uint8(reshape(img_vals,rows,cols,comps)),[])
            [idxs,centers,radii] = RGBLandmarkEstimator_Student.findColoredSpheres( ...
                img_rgb,landmark_colors);
            for lidx=1:length(idxs)
                idx=idxs(lidx);
                xy_center = centers(lidx,:);
                radius = radii(lidx);
                [x,y]=StateRenderer.makeCircle(radius, 10, xy_center);
                figure(1), hold on, plot(x,y,'Color',landmark_colors(idx,:), ...
                    'LineWidth',2);
            end
            aaa=1
        end
        
        function [idxs, centers, radii] = findColoredSpheres(img_rgb, landmark_colors)
            num_colors = size(landmark_colors,1);
            row_start = 150;
            row_end = 300;
            idxs=[];
            centers=[];
            radii=[];
            if (1==0)
                [centers, radii] = imfindcircles(img_rgb,[7 15], ...
                    'ObjectPolarity','bright', 'Sensitivity',0.9);
                idxs=ones(1,length(radii));
                return;
            elseif (1==0)
                img_cv=rgb2hsv(img_rgb(row_start:row_end,:,:));
                [rows, cols, ~]=size(img_cv);
                candidate_hsv = rgb2hsv(landmark_colors);
                candidate_hsv = candidate_hsv(:,1:2);
                img_vals = reshape(img_cv(:,:,1:2),rows*cols,2);
                projVals = img_vals*candidate_hsv';
                [projMax, labelVals] = max(projVals');                
            elseif (1==0)
                img_cv=rgb2lab(img_rgb(row_start:row_end,:,:));
                [rows, cols, ~]=size(img_cv);
                candidate_lab = rgb2lab(landmark_colors);
                img_vals = reshape(img_cv(:,:,1:3),rows*cols,3);
                projVals = img_vals(:,1:2)*candidate_lab(:,1:2)';
                [projMax, labelVals] = max(projVals');
            elseif (1==1)
                img_cv=img_rgb(row_start:row_end,:,:);
                [rows, cols, ~]=size(img_cv);
                candidate_rgb = landmark_colors;
                img_vals = reshape(img_cv(:,:,1:3),rows*cols,3);
                projVals = max(img_vals,[],2)-min(img_vals,[],2);
                THRESHOLD=40;
                colorIdxs = find(projVals > THRESHOLD);
                labelVals = zeros(rows*cols,1);
                for pixIdx=1:length(colorIdxs)
                    idx = colorIdxs(pixIdx);
                    pixval = double(img_vals(idx,:))/255;
                    pixval = pixval - min(pixval);
                    [~, labelIdx] = max(pixval*candidate_rgb');                   
                    labelVals(idx) = labelIdx;
                end
            else
                img_cv=img_rgb(row_start:row_end,:,:);
                [rows, cols, comps]=size(img_cv);
                img_vals = reshape(img_cv,rows*cols,comps);
                img_vals = double(img_vals)/255;
                meanInt = mean(img_vals);
                img_vals = double(img_vals) - ones(rows*cols,1)*meanInt;
                landmark_colors = landmark_colors - ones(num_colors,1)*meanInt;
                projVals = img_vals*landmark_colors';
                [projMax, labelVals] = max(projVals');
            end
            
            img_seg = reshape(labelVals,rows,cols);
            if (false)
                figure(2), imagesc(img_seg);
                axis equal;
                hold on;
            end
            
            for labelVal=1:num_colors
                [y,x]=find(labelVal==img_seg);
                numpts=length(y);
                if (numpts > 30)
                    xy_center = mean([x y]);
                    zero_ctr = [x y]-ones(numpts,1)*xy_center;
                    covMat = (1/(numpts-1)) * (zero_ctr'*zero_ctr);
                    eigVals = eig(covMat);
                    ecc = abs(1-eigVals(1)/eigVals(2));
                    %fprintf(1,'Detected class %d eigvals (%f,%f) ecc %f\n', ...
                    %    labelVal, eigVals(1), eigVals(2), ecc);
                    if (ecc < 0.4)
                       radius = 2*sqrt(max(eigVals));
                       idxs=[idxs; labelVal];
                       centers=[centers; xy_center+[0,row_start-1]];
                       radii=[radii; radius];
                    end
                end
            end
        end        
    end
    
    methods
        function obj = RGBLandmarkEstimator_Student()
            obj@RGBCameraListener();
        end
        
        function setLandmarkColors(obj, landmarkColors)
            obj.landmarkColors = landmarkColors;
        end
        
        function setLandmarkPositions(obj, landmarkPositions)
            obj.landmarkPositions = landmarkPositions;
        end
        
        function setLandmarkDiameter(obj, diameter)
            obj.landmarkDiameter = diameter;
        end
        
        function setPublisher(obj, topic)
            obj.landmarkPublisher = rospublisher(topic,'geometry_msgs/PointStamped');
        end
                
        function processImage(obj, imgRGB, tfmgr, tstamp)
            global GUI;
            RGBCameraListener.showRGBImage(imgRGB);
            [idxs,centers,radii] = RGBLandmarkEstimator_Student.findColoredSpheres( ...
                imgRGB, obj.landmarkColors);
            GUI.setFigure('IMAGE')
            for lidx=1:length(idxs)
                idx=idxs(lidx);
                xy_center = centers(lidx,:);
                radius = radii(lidx);
                [x,y]=StateRenderer.makeCircle(radius, 10, xy_center);
                hold on, plot(x,y,'Color', obj.landmarkColors(idx,:), ...
                    'LineWidth', 0.5);                
                if (~isempty(obj.Pmatrix))
                    focal_lengths = [obj.Pmatrix(1,1), obj.Pmatrix(2,2)];
                    principal_pt =[obj.Pmatrix(1,3), obj.Pmatrix(2,3)];                   
                    physical_diameter_m = obj.landmarkDiameter;
                    phi = 0;
                    radius_m = 0;
                    fprintf('Landmark %d detected at (Radius,Heading) (%0.2f m., %0.2f degrees)\n', ...
                        idx, radius_m, phi*180/pi);
                    if (tfmgr.tftree.canTransform('map', 'base_link_truth'))
                        tfmgr.tftree.waitForTransform('map', 'base_link_truth');
                        map2basetf = tfmgr.tftree.getTransform('map', 'base_link_truth');
                        tVal = map2basetf.Transform.Translation;
                        qVal = map2basetf.Transform.Rotation;
                        pose = LocalPose([tVal.X tVal.Y tVal.Z], ...
                            [qVal.W qVal.X qVal.Y qVal.Z]);
                        rpy=PurePursuitController_Student.quat2rpy(pose.qorientation);
                        yawAngle = rpy(3);
                        %fprintf('State is (%0.2f,%0.2f,%0.2f)\n', ...
                        %    tVal.X, tVal.Y, yawAngle*180/pi);
                        landmarkPos = obj.landmarkPositions(idx,:);
                        actual_phi = atan2(landmarkPos(2)-tVal.Y, landmarkPos(1)-tVal.X) - yawAngle;
                        actual_radius_m = norm(landmarkPos-[tVal.X, tVal.Y]);
                        fprintf('Landmark %d actually at (Radius,Heading) (%0.2f m., %0.2f degrees)\n', ...
                            idx, actual_radius_m, actual_phi*180/pi);
                        fprintf('Landmark %d error (%0.2f m., %0.2f degrees)\n', ...
                            idx, radius_m-actual_radius_m, (phi-actual_phi)*180/pi);
                        if (~isempty(obj.landmarkPublisher))
                            landmarkMsg = rosmessage(obj.landmarkPublisher);
                            landmarkMsg.Point.X = actual_radius_m;
                            landmarkMsg.Point.Y = actual_phi;
                            landmarkMsg.Point.Z = idx;
                            landmarkMsg.Header.Stamp = tstamp;
                            obj.landmarkPublisher.send(landmarkMsg);
                            disp('landmark published');
                        end
                    end                    
                end
            end
            %obj.saveImageData(imgRGB);            
        end        
        
    end
end

