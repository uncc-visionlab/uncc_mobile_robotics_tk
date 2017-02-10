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
classdef OdometryPathRecorder < OdometryListener
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        odom_pathPts
        actual_pathPts
        recorded_times
        MAX_VALUES=10000
        numPathPts
    end
    
    methods
        function obj = OdometryPathRecorder(stateProvider)
            global GAZEBO_SIM;
            obj@OdometryListener(stateProvider);
            obj.odom_pathPts=zeros(obj.MAX_VALUES,3);
            if (GAZEBO_SIM==true)
                obj.actual_pathPts=zeros(obj.MAX_VALUES,3);
            end
            obj.recorded_times = zeros(obj.MAX_VALUES,1);
            obj.numPathPts=0;
        end
        
        function odometryCallback(obj, varargin)
            global GAZEBO_SIM START_TIME;
            obj.odometryCallback@OdometryListener(varargin{:});
            obj.numPathPts = mod(obj.numPathPts,obj.MAX_VALUES);
            obj.numPathPts = obj.numPathPts + 1;
            obj.odom_pathPts(obj.numPathPts,:) = obj.odom_position;
            if (GAZEBO_SIM==true)
                obj.actual_pathPts(obj.numPathPts,:) = obj.actual_position;
            end
            duration = rostime('now')-START_TIME;
            duration_secs = duration.Sec+duration.Nsec*10^-9;
            obj.recorded_times(obj.numPathPts)=duration_secs;            
        end
    end
    
end

