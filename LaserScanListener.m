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
classdef LaserScanListener < handle
    properties
        %laserScanTopic
        laserScanSub
        laserScanTimer
        tf_baseNode
        failedMsgCount
        MAX_BAD_MSGS        
    end
    
    methods (Static)
        function plotScan(laserScanMessage, pose)
            global START_TIME;
            global GUI;
            
            persistent linehandle;
            ranges = laserScanMessage.Ranges';
            angles = laserScanMessage.AngleMin: ...
                laserScanMessage.AngleIncrement:laserScanMessage.AngleMax;
            %angles = -angles;
            xVals=ranges.*cos(angles);
            yVals=ranges.*sin(-angles);
            if (exist('pose','var')==1)
                [xVals, yVals] = pose.transform(xVals, yVals);
            end
            if (isempty(linehandle)==0)
                % remove previous scan plot
                delete(linehandle);
            end
            %figure(1);
            GUI.setFigure('MAP');
            linehandle = plot(xVals,yVals,'.','MarkerSize',5);            
            duration = rostime('now')-START_TIME;
            duration_secs = duration.Sec+duration.Nsec*10^-9;
            %uicontrol('Style', 'text', ...
            %    'String', datestr(timeStruct.Data.time), ...
            %    'Units','normalized', ...
            %    'Position', [0.3 0.1 0.4 0.1]);
            %text(100,100,datestr(timeStruct.Data.time),'Color','red','FontSize',14);
            xlabelstr = sprintf('laser scan timestamp: %0.3f',duration_secs);
            xlabel(xlabelstr);
        end
    end
    
    methods
        function obj = LaserScanListener(topicstr)
            obj.laserScanSub = rossubscriber(topicstr,'BufferSize',10);
            obj.tf_baseNode = 'base_link';
            obj.failedMsgCount = 0;
            obj.MAX_BAD_MSGS = 5;            
        end
        
        function setCallbackRate(obj, rate, tfmgr)
            if (strcmpi(rate,'fastest')==1)
                obj.laserScanSub.NewMessageFcn = ...
                    {@obj.laserScanCallback, tfmgr};
            else
                obj.laserScanTimer = timer('TimerFcn', ...
                    {@obj.laserScanCallback, obj.laserScanSub, tfmgr}, ...
                    'Period',rate,'ExecutionMode','fixedSpacing');
                pause(2);
                start(obj.laserScanTimer);
            end
        end
        
        function laserScanCallback(obj, varargin)            
            if (isa(varargin{1},'timer')==1)
                laserScanMessage = varargin{3}.LatestMessage;
                tfmgr = varargin{4};
            else
                laserScanMessage = varargin{1}.LatestMessage;
                tfmgr = varargin{2};
            end
            
            if (isempty(laserScanMessage))
                disp('LaserScanListener::Skipping empty laser scan message.');
                obj.failedMsgCount = obj.failedMsgCount + 1;
                if (obj.failedMsgCount > obj.MAX_BAD_MSGS)
                    delete obj;
                else
                    return;
                end
            else
                obj.failedMsgCount = 0;                
            end

            if (tfmgr.tftree.canTransform('map', obj.tf_baseNode))
                tfmgr.tftree.waitForTransform('map', obj.tf_baseNode)
                map2basetf = tfmgr.tftree.getTransform('map', obj.tf_baseNode);%, ...
                scanTime = laserScanMessage.Header.Stamp.Sec+laserScanMessage.Header.Stamp.Nsec*10^-9;
                tfTime = map2basetf.Header.Stamp.Sec+map2basetf.Header.Stamp.Nsec*10^-9;
                %rostime('now') - rosduration(0.2), 'Timeout', 2);
                %currentTime = rostime('now');
                %time_diff = currentTime.Sec - scanTime.Sec
                %if (currentTime.Sec - scanTime.Sec > 10)
                %    disp('Discrepancy in header time.');
                %    return;
                %end
                %map2basetf = tfmgr.tftree.getTransform('map', 'base_link', ...
                %    laserScanMessage.Header.Stamp, 'Timeout', 2)
                time_diff = abs(scanTime-tfTime);
                if (time_diff < 0.1)
                    tVal = map2basetf.Transform.Translation;
                    qVal = map2basetf.Transform.Rotation;
                    pose = LocalPose([tVal.X tVal.Y tVal.Z], ...
                        [qVal.W qVal.X qVal.Y qVal.Z]);
                    LaserScanListener.plotScan(laserScanMessage, pose);
                else
                    %disp('LaserScanListener could not get up-to-date map->base_link transform');
                end
                
            else
                disp('LaserScanListener could not get map->base_link transform');
            end
        end
    end
end