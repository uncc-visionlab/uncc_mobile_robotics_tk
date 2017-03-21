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
classdef KobukiSim < ExampleHelperGazeboSpawnedModel
    % Kobuki This class handles the kobuki/turtlebot as an object. It can
    % be used to access the sensor suite of the robot and it holds
    % references to all of the objects that it "contains." For example it
    % holds references to it's camera, laser scan, and odometry subscriber
    % objects which allows multiple turtlebots to be instantiated. Each 
    % instance will hold unique references to their sensor suites reducing
    % complexity of higher level code for multi-robot contexts.
    
    properties
        world_gazebo
        
        laserScanListener
        rgbCamListener        
        odometryListener
        odometryEKF
        
        velocityController
    end
    
    methods
        function obj = KobukiSim(gazebo)
            obj@ExampleHelperGazeboSpawnedModel('mobile_base',gazebo);
            obj.world_gazebo = gazebo;
            obj.laserScanListener = LaserScanListener('/hokuyo_scan');
            %obj.rgbCamListener = RGBCameraListener();
            %obj.rgbCamListener = RGBLandmarkEstimator();
            obj.rgbCamListener = RGBLandmarkEstimator_Student();
            %obj.odometryListener = OdometryPathRecorder(obj);
            obj.odometryListener = OdometryListener(obj);
            %obj.velocityController = LaserScanAvoidController();
            %obj.odometryEKF = OdomEstimationNode(true, true, false, false);
            %obj.velocityController = PurePursuitController(obj);
            obj.velocityController = PurePursuitController_Student(obj);
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