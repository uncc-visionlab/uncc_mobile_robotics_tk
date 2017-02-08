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
    end
    
    methods (Static)
        function showRGBImage(img, pose)
            global START_TIME;
            
            numpixels=length(img.Data);
            r=img.Data(1:3:numpixels);
            g=img.Data(2:3:numpixels);
            b=img.Data(3:3:numpixels);
            r=reshape(r,img.Width,img.Height)';
            g=reshape(g,img.Width,img.Height)';
            b=reshape(b,img.Width,img.Height)';
            imgRGB=cat(3,r,g,b);
            figure(2);
            imshow(imgRGB);

            duration = rostime('now')-START_TIME;
            duration_secs = duration.Sec+duration.Nsec*10^-9;
            figure(2);
            xlabelstr = sprintf('Image timestamp: %0.3f',duration_secs);
            xlabel(xlabelstr);
        end
    end
    
    methods
        function obj = RGBCameraListener()
            obj.rgbCamSub = rossubscriber('/camera/rgb/image_raw','BufferSize',1);
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
            if (tfmgr.tftree.canTransform('map', 'base_link'))
                tfmgr.tftree.waitForTransform('map', 'base_link');
                map2basetf = tfmgr.tftree.getTransform('map', 'base_link');
                tVal = map2basetf.Transform.Translation;
                qVal = map2basetf.Transform.Rotation;
                pose = LocalPose([tVal.X tVal.Y tVal.Z], ...
                    [qVal.W qVal.X qVal.Y qVal.Z]);
                RGBCameraListener.showRGBImage(rgbCamMessage, pose);
            else
                disp('RGBCameraListener could not get map->base_link transform');
            end
        end
    end
end