classdef StateRenderer_Gaussian < StateRenderer
    %StateRenderer_Gaussian Visualizes the (x,y) pose state uncertainty
    % by drawing the 2-sigma confidence region for the true pose
    
    properties
        % polygon for the circle
        ellipsePolyPoints
        ellipsehandle
    end
    
    methods
        
        function obj = StateRenderer_Gaussian()
            obj@StateRenderer();
            [x,y] = StateRenderer.makeCircle(obj.baseRadius, 12, [0,0]);
            obj.ellipsePolyPoints(1,:) = x;
            obj.ellipsePolyPoints(2,:) = y;
            obj.ellipse_color = [0 0 0]; % black
        end
        
        function showState(obj, position, qorientation, covariance)
            global GUI;
            obj.showState@StateRenderer(position, qorientation);
                        
            if (~exist('covariance','var'))
                return;
            end
            if (~isempty(obj.ellipsehandle))
                % remove previous scan robot plot
                delete(obj.ellipsehandle);
            end
            % draw 2-sigma uncertainty ellipse
            xy_cov = [covariance(1,1) covariance(1,2); ...
                covariance(2,1) covariance(2,2)];
            [evecs, evals] = eig(xy_covariance)
            x = obj.ellipsePolyPoints(1,:)*2*sqrt(evals(1,1));
            y = obj.ellipsePolyPoints(2,:)*2*sqrt(evals(2,2));
            GUI.setFigure('MAP');
            rotmat = [evecs [0; 0]; 0 0 1];
            qorientation = QuatLib.mat2quat(rotmat);
            localpose = LocalPose(position, qorientation);            
            [ox, oy] = localpose.transform(obj.ellipsePolyPoints(1,:), ...
                obj.ellipsePolyPoints(2,:));
            obj.ellipsehandle = plot(ox,oy,'Color',obj.ellipse_color);                
        end
    end
    
end

