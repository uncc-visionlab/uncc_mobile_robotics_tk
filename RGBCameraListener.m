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
classdef RGBCameraListener < handle
    properties
        %rgbCamTopic
        rgbCamSub
        rgbCamTimer
        rgbCamInfoSub
        distortionCoeffs
        Kmatrix
        Pmatrix
        latestImage
        latestPose
        tf_baseNode
    end
    
    methods (Static)
        function imgRGB = convertRGBImageMessage(imgMsg)
            imgRGB=[];
            if (prod(size(imgMsg))==0)
                return;
            end
            numpixels=length(imgMsg.Data);
            r=imgMsg.Data(1:3:numpixels);
            g=imgMsg.Data(2:3:numpixels);
            b=imgMsg.Data(3:3:numpixels);
            r=reshape(r,imgMsg.Width,imgMsg.Height)';
            g=reshape(g,imgMsg.Width,imgMsg.Height)';
            b=reshape(b,imgMsg.Width,imgMsg.Height)';
            imgRGB = cat(3,r,g,b);
        end
        
        function showRGBImage(imgRGB)
            global START_TIME GUI;
            GUI.setFigure('IMAGE')
            imshow(imgRGB);

            duration = rostime('now')-START_TIME;
            duration_secs = duration.Sec+duration.Nsec*10^-9;
            
            GUI.setFigure('IMAGE')
            xlabelstr = sprintf('Image timestamp: %0.3f',duration_secs);
            xlabel(xlabelstr);
        end        
    end
    
    methods
        function obj = RGBCameraListener()
            obj.rgbCamSub = rossubscriber('/camera/rgb/image_raw','BufferSize',1);
            obj.tf_baseNode = 'base_link';
        end
        
        function getCameraInfo(obj) 
            if (size(obj.rgbCamInfoSub,1)==0)
                obj.rgbCamInfoSub = rossubscriber('/camera/rgb/camera_info','BufferSize',1);
            end
            cameraInfoMsg = receive(obj.rgbCamInfoSub);
            obj.distortionCoeffs = cameraInfoMsg.D;
            obj.Kmatrix = reshape(cameraInfoMsg.K,3,3)';
            obj.Pmatrix = reshape(cameraInfoMsg.P,4,3)';
        end
        
        function setCallbackRate(obj, rate, tfmgr)
            if (strcmpi(rate,'fastest')==1)
                obj.rgbCamSub.NewMessageFcn = ...
                    {@obj.rgbImageCallback, tfmgr};
            else
                obj.rgbCamTimer = timer('TimerFcn', ...
                    {@obj.rgbImageCallback, obj.rgbCamSub, tfmgr}, ...
                    'Period',rate,'ExecutionMode','fixedSpacing');
                pause(2);
                start(obj.rgbCamTimer);
            end
        end
        
        function rgbImageCallback(obj, varargin)
            if (isa(varargin{1},'timer')==1)
                rgbCamMessage = varargin{3}.LatestMessage;
                tfmgr = varargin{4};
            else
                rgbCamMessage = varargin{1}.LatestMessage;
                tfmgr = varargin{2};
            end
            if (tfmgr.tftree.canTransform('map', obj.tf_baseNode))
                tfmgr.tftree.waitForTransform('map', obj.tf_baseNode);
                map2basetf = tfmgr.tftree.getTransform('map', obj.tf_baseNode);
                tVal = map2basetf.Transform.Translation;
                qVal = map2basetf.Transform.Rotation;
                pose = LocalPose([tVal.X tVal.Y tVal.Z], ...
                    [qVal.W qVal.X qVal.Y qVal.Z]);
                imgRGB = RGBCameraListener.convertRGBImageMessage(rgbCamMessage);
                obj.latestPose = pose;
                if ~isempty(imgRGB)
                    obj.latestImage = imgRGB;
                end                
                obj.processImage(imgRGB);
            else
                disp('RGBCameraListener could not get map->base_link transform');
            end
        end
               
        function processImage(obj, imgRGB)
            RGBCameraListener.showRGBImage(imgRGB);
            obj.saveImageData(imgRGB);
        end
        
        function saveImageData(obj, imgRGB)
            imageInfo.img = imgRGB;
            imageInfo.distortionCoeffs = obj.distortionCoeffs;
            imageInfo.Kmatrix = obj.Kmatrix;
            imageInfo.Pmatrix = obj.Pmatrix;
            disp('Saving image data to file imageData.mat');
            save('imageData.mat','imageInfo');
        end
    end
end