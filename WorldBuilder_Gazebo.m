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
classdef WorldBuilder_Gazebo < ExampleHelperGazeboCommunicator
    properties
        keep_models    % list of 3D model names to keep
                       % when the world is cleared of objects        
    end
    methods (Static)
        function demo()
            clear;
            close all;
            clc;
            %ipaddress = '10.38.46.59';            
            %ipaddress = '192.168.11.178';           
            ipaddress = '10.16.30.15';           
            if (robotics.ros.internal.Global.isNodeActive==0)
                rosinit(ipaddress)
            end
            world = WorldBuilder_Gazebo();
            wall = ExampleHelperGazeboModel('grey_wall','gazeboDB');
            wallDimensions = [7.5, .2, 2.8];
            wallLength = wallDimensions(1);
            world.spawnModel(wall,[wallLength/2 0 0],[0 0 pi/2]);
            world.spawnModel(wall,[-wallLength/2 0 0],[0 0 pi/2]);
            world.spawnModel(wall,[0 -wallLength/2 0],[0 0 0]);
            world.spawnModel(wall,[0 wallLength/2 0],[0 0 0]);
        end
    end
    methods
        function obj = WorldBuilder_Gazebo()
            obj@ExampleHelperGazeboCommunicator();
            obj.keep_models{1}='ground_plane';
            obj.keep_models{2}='mobile_base';
        end
        
        function object = createSharedObject(obj, obj_name, pos, orient)
            if (exist('pos','var')==0)
                pos = [0 0 0];
            end
            if (exist('orient','var')==0)
                orient = [0 0 0];
            end
            object = ExampleHelperGazeboModel(obj_name,'gazeboDB',pos,orient);
            obj.gazebo.spawnModel(object);
        end
        
        function removeAllTemporaryModels(gazebo)
            list = getSpawnedModels(gazebo);
            for idx=1:length(list)
                modelName=list{idx};
                if (~ismember(list{idx},gazebo.keep_models))
                    try
                        removeModel(gazebo,list{idx});
                    catch
                        disp(['Error while removing' modelName]);
                    end
                end
            end
        end
    end
end