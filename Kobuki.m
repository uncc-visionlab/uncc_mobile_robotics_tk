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
classdef Kobuki < handle
    % Kobuki This class handles the kobuki/turtlebot as an object. It can
    % be used to access the sensor suite of the robot and it holds
    % references to all of the objects that it "contains." For example it
    % holds references to it's camera, laser scan, and odometry subscriber
    % objects which allows multiple turtlebots to be instantiated. Each 
    % instance will hold unique references to their sensor suites reducing
    % complexity of higher level code for multi-robot contexts.
    
    properties
        
        laserScanListener
        rgbCamListener        
        odometryListener
        localizationEKF
        
        velocityController
    end
    
    methods
        function obj = Kobuki(namespace)
            if (exist('namespace','var')==0 || isempty(namespace))
                namespace = 'mobile_base';                
            end
            if (strcmp(namespace,'mobile_base'))
                namespace=[];
            end            
            %obj.laserScanListener = LaserScanListener('/scan');
            
            %obj.rgbCamListener = RGBCameraListener(namespace);
            %obj.rgbCamListener = RGBLandmarkEstimator(namespace);
            obj.rgbCamListener = RGBLandmarkEstimator_Student(namespace);
            %obj.rgbCamListener = RGBLandmarkEstimatorAdvanced(namespace);

            %obj.odometryListener = OdometryPathRecorder(obj, namespace);
            %obj.odometryListener = OdometryListener(obj);
            
            %obj.velocityController = LaserScanAvoidController();            
            obj.velocityController = PurePursuitController(obj, namespace);
            %obj.velocityController = PurePursuitController_Student(obj, namespace);
            %obj.velocityController = PIDController(obj, namespace, bagfile);

            %obj.localizationEKF = EKF_Localization(namespace);
            %obj.localizationEKF = EKF_Localization_Student(namespace);
            %obj.localizationEKF = EKF_SLAM_Student(namespace);
            obj.localizationEKF = EKF_SLAM(namespace);
        end
                        
        function sendVelocityCommand(kobuki, velocityMsg)
            if (size(kobuki.velocityController.velocityPub,1)==0)
                kobuki.velocityController.velocityPub = rospublisher('/mobile_base/commands/velocity');
            end
            send(kobuki.velocityController.velocityPub, ...
                velocityMsg);
        end
        
        function scanMsg = getLaserScan(kobuki)
            if (size(kobuki.laserScanListener.laserScanSub,1)==0)
                kobuki.laserScanListener.laserScanSub = rossubscriber('/hokuyo_scan','BufferSize',1);
            end
            scanMsg = receive(kobuki.laserScanListener.laserScanSub);
        end
        
        function imgRGB = getCameraImage(kobuki)
            if (size(kobuki.rgbCamListener.rgbCamSub,1)==0)
                kobuki.rgbCamListener.rgbCamSub = rossubscriber('/camera/rgb/image_raw','BufferSize',1);
            end
            img = receive(kobuki.rgbCamListener.rgbCamSub);
            numpixels=length(img.Data);
            r=img.Data(1:3:numpixels);
            g=img.Data(2:3:numpixels);
            b=img.Data(3:3:numpixels);
            r=reshape(r,img.Width,img.Height)';
            g=reshape(g,img.Width,img.Height)';
            b=reshape(b,img.Width,img.Height)';
            imgRGB=cat(3,r,g,b);
        end        
    end
end