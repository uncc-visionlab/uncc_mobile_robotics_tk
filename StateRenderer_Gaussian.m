classdef StateRenderer_Gaussian < StateRenderer
    %StateRenderer_Gaussian Visualizes the (x,y) pose state uncertainty
    % by drawing the 2-sigma confidence region for the true pose
    
    properties
        % polygon for the circle
        ellipsePolyPoints
        ellipse_color
        ellipsehandle
    end
    methods (Static)
        function test
            global GUI;
            GUI = ROSGUI();
            h = GUI.getFigure('MAP');
            set(h,'Visible','on');
            sr = StateRenderer_Gaussian();
            position=[0 0 0];
            qorientation=[1 0 0 0];
            covariance=zeros(3,3);
            covariance(1,1) = .7;
            covariance(2,2) = .3;
            theta = -pi/4;
            R=[cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
            covariance = R'*covariance*R;
            sr.showState(position, qorientation, covariance);
        end
    end
    methods
        
        function obj = StateRenderer_Gaussian()
            obj@StateRenderer();
            [x,y] = StateRenderer.makeCircle(1, 24, [0,0]);
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
            [evecs, evals] = eig(xy_cov);
            x = obj.ellipsePolyPoints(1,:)*2*sqrt(evals(1,1));
            y = obj.ellipsePolyPoints(2,:)*2*sqrt(evals(2,2));
            GUI.setFigure('MAP');
            rotmat = [evecs' [0; 0]; 0 0 det(evecs)];
            qorientation = QuatLib.mat2quat(rotmat);
            localpose = LocalPose(position, qorientation);            
            [ox, oy] = localpose.transform(x, y);
            obj.ellipsehandle = plot(ox,oy,'Color',obj.ellipse_color);                
        end
    end
    
end

