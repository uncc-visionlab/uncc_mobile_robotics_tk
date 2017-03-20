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
classdef StateRenderer < handle
    % StateRenderer This class draws the current state in a plot.
    %       
    properties
        % radius of the robot base
        baseRadius
        % polygon for the robot boundary
        boundaryPolyPoints
        % polygon for the orientation
        orientationPolyPoints
        % handles to the robot state drawn curves 
        basehandle
        arrowhandle
        body_color         % color of the robot body
        arrow_color        % color of the orientation arrow
    end
    
    methods (Static)
        function [x,y] = makeCircle(radius, numVerts, center)
            if ~exist('center','var')
                center=[0 0];
            end
            delta_theta=2*pi/numVerts; % angle increment for Kobuki base polygon
            for thetaIdx=0:numVerts
                theta = thetaIdx*delta_theta;
                x(thetaIdx+1)=radius*cos(theta)+center(1);
                y(thetaIdx+1)=radius*sin(theta)+center(2);
            end            
        end
    end
    
    methods
        function obj = StateRenderer()
            % setup the polygons to draw the Kobuki base
            % polygons are a circle and an arrow showing orientation
            obj.baseRadius=0.2; % radius of Kobuki base
            [x,y] = StateRenderer.makeCircle(obj.baseRadius, 12, [0,0]);
            obj.boundaryPolyPoints(1,:) = x;
            obj.boundaryPolyPoints(2,:) = y;
%            delta_theta=pi/6; % angle increment for Kobuki base polygon
%            numVerts=2*pi/delta_theta;
%            obj.boundaryPolyPoints=zeros(2,numVerts);
%            for thetaIdx=0:numVerts
%                theta = thetaIdx*delta_theta;
%                obj.boundaryPolyPoints(1,thetaIdx+1)=obj.baseRadius*cos(theta);
%                obj.boundaryPolyPoints(2,thetaIdx+1)=obj.baseRadius*sin(theta);
%            end
            obj.orientationPolyPoints = [ ...
                -0.6609 -0.6609 0.1437 0.1437 0.7471  0.1437    0.1437 -0.6609;
                -0.2012  0.2012 0.2012 0.4023 0      -0.4023   -0.2012 -0.2012];
            obj.orientationPolyPoints=obj.orientationPolyPoints*0.8*obj.baseRadius;
            obj.body_color = [0 0 1]; % blue 'b'
            obj.arrow_color = [1 0 0]; % red 'r'           
        end
        
        function showState(obj, position, qorientation)
            global GUI;
            
            if (isempty(obj.basehandle)==0)
                % remove previous scan robot plot
                delete(obj.basehandle);
            end
            if (isempty(obj.arrowhandle)==0)
                % remove previous scan orientation plot
                delete(obj.arrowhandle);
            end
            localpose = LocalPose(position,qorientation);
            %figure(1);
            GUI.setFigure('MAP');
            [ox, oy] = localpose.transform(obj.boundaryPolyPoints(1,:), ...
                obj.boundaryPolyPoints(2,:));
            obj.basehandle = plot(ox,oy,'Color',obj.body_color);
            [ox, oy] = localpose.transform(obj.orientationPolyPoints(1,:), ...
                obj.orientationPolyPoints(2,:));         
            obj.arrowhandle = plot(ox,oy,'Color', obj.arrow_color);
        end
    end    
end

