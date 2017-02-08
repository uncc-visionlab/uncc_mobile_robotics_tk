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
classdef LocalPose < handle
    properties
        position
        qorientation   
    end
    
    methods
        function obj = LocalPose(position, qorientation)
            obj.position = position;
            obj.qorientation = qorientation;
        end
        
        function setPose(obj, position, qorientation)
            if (isempty(position)==0)
                obj.position = position;
            end
            if (isempty(qorientation)==0)
                obj.qorientation = qorientation;
            end
        end
        
        function [ox, oy, oz] = transform(obj, x, y, z)            
            rotMat=quat2rotm(obj.qorientation);
            if (exist('z','var')==0)
                rotMat2D=rotMat(1:2,1:2);
                numPts=size(x,2);
                p2d=[x; y];
                o_p2d = rotMat2D*p2d;
                o_p2d = o_p2d + ...
                    obj.position(1:2)'*ones(1,numPts);
                ox = o_p2d(1,:);
                oy = o_p2d(2,:);
            end
        end        
    end
end