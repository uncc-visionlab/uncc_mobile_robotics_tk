classdef QuatLib
    %QuatLib A Library of quaternion functions
    
    properties 
    end
    
    methods (Static)
        function test
            theta = pi/4;
            rotmat = [cos(theta) -sin(theta) 0; ...
                sin(theta) cos(theta) 0; ...
                0 0 1];
            rpyA = QuatLib.mat2rpy(rotmat);
            quatA = QuatLib.mat2quat(rotmat);
            quatB = QuatLib.rpy2quat(rpyA);
            rpyB = QuatLib.quat2rpy(quatA);
            matA = QuatLib.rpy2mat(rpyB);
            matB = QuatLib.quat2mat(quatB);
            err = sum(rpyA-rpyB,2)+sum(quatA-quatB,2)+sum(sum(matA-matB));
            fprintf(1,'Conversion error is %0.4f\n',err);
        end
                
        function q = rpy2quat(rpy)
            %Converts rpy body 321 sequence (yaw pitch roll) to quaternion
            %q = q0 + q1 i + q2 j + q3 k
            r = rpy(1);
            p = rpy(2);
            y = rpy(3);
            cy = cos(y/2);
            sy = sin(y/2);
            cp = cos(p/2);
            sp = sin(p/2);
            cr = cos(r/2);
            sr = sin(r/2);
            
            q = [cr*cp*cy + sr*sp*sy, ...
                sr*cp*cy - cr*sp*sy, ...
                cr*sp*cy + sr*cp*sy, ...
                cr*cp*sy - sr*sp*cy];
        end
                
        function mat = rpy2mat(rpy)
            r = rpy(1);
            p = rpy(2);
            y = rpy(3);
            cy = cos(y);
            sy = sin(y);
            cp = cos(p);
            sp = sin(p);
            cr = cos(r);
            sr = sin(r);
            
            mat = [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr;
                sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr;
                -sp, cp*sr, cp*cr];
        end
        
        function q = mat2quat(mat)
            % MAT2QUAT - Convert a 3x3 rotation matrix to a quaternion
            %
            
            %quat = rotm2quat(mat);
            Quat=zeros(4,1);
            
            %Compute absolute values of the four quaternions from diags of Eqn 12-13
            %absQ=0.5*sqrt([1 -1 -1;-1 1 -1;-1 -1 1;1 1 1]*diag(mat)+1);
            
            absQ(1,:) = 0.5 * sqrt(1+mat(1,1,:)+mat(2,2,:)+mat(3,3,:));
            absQ(2,:) = 0.5 * sqrt(1-mat(1,1,:)-mat(2,2,:)+mat(3,3,:));
            absQ(3,:) = 0.5 * sqrt(1+mat(1,1,:)-mat(2,2,:)-mat(3,3,:));
            absQ(4,:) = 0.5 * sqrt(1-mat(1,1,:)+mat(2,2,:)-mat(3,3,:));
            
            [~,ind]=max(absQ); % Select biggest for best accuracy
            
            qind = ind == 1;
            if any(qind)
                Quat(1,qind)=absQ(1,qind);
                Quat(2,qind)=squeeze((mat(2,3,qind)-mat(3,2,qind))).'.*0.25./absQ(1,qind);
                Quat(3,qind)=squeeze((mat(3,1,qind)-mat(1,3,qind))).'.*0.25./absQ(1,qind);
                Quat(4,qind)=squeeze((mat(1,2,qind)-mat(2,1,qind))).'.*0.25./absQ(1,qind);
            end
            
            qind = ind == 2;
            if any(qind)
                Quat(1,qind)=squeeze(mat(1,2,qind)-mat(2,1,qind)).'.*0.25./absQ(2,qind);
                Quat(2,qind)=squeeze(mat(3,1,qind)+mat(1,3,qind)).'.*0.25./absQ(2,qind);
                Quat(3,qind)=squeeze(mat(3,2,qind)+mat(2,3,qind)).'.*0.25./absQ(2,qind);
                Quat(4,qind)=absQ(2,qind);
            end
            
            qind = ind == 3;
            if any(qind)
                Quat(1,qind)=squeeze(mat(2,3,qind)-mat(3,2,qind)).'.*0.25./absQ(3,qind);
                Quat(2,qind)=absQ(3,qind);
                Quat(3,qind)=squeeze(mat(1,2,qind)+mat(2,1,qind)).'.*0.25./absQ(3,qind);
                Quat(4,qind)=squeeze(mat(3,1,qind)+mat(1,3,qind)).'.*0.25./absQ(3,qind);
            end
            
            qind = ind == 4;
            if any(qind)
                Quat(1,qind)=squeeze(mat(3,1,qind)-mat(1,3,qind)).'.*0.25./absQ(4,qind);
                Quat(2,qind)=squeeze(mat(1,2,qind)+mat(2,1,qind)).'.*0.25./absQ(4,qind);
                Quat(3,qind)=absQ(4,qind);
                Quat(4,qind)=squeeze(mat(2,3,qind)+mat(3,2,qind)).'.*0.25./absQ(4,qind);
            end
            
            q = real(Quat);
            q = q./norm(q);
            q = q';
        end
        
        function rpy = mat2rpy(mat)
            %rpy = QuatLib.quat2rpy(QuatLib.mat2quat(mat));                        
            rpy = [atan2(mat(3,2), mat(3,3)), ...
                atan2(-mat(3,1), sqrt(mat(3,2)*mat(3,2) + mat(3,3)*mat(3,3))), ...
                atan2(mat(2,1), mat(1,1))];
        end
        
        function rpy = quat2rpy(q)
            %QUAT2RPY Converts the quaternion in the form(q0 + q1 i + q2 j + q3 k into the roll pitch yaw
            %pitch yaw (ZYX convention) other conventions can be supported in later
            %versions. q is nx4 matrix output in radians
            if size(q,2) ~= 4
                q = q';
            end
            rpy(:,1) = atan2(2*(q(:,1).*q(:,2) +q(:,3).*q(:,4)), 1 - 2*(q(:,2).^2 + q(:,3).^2));
            rpy(:,2) = asin(2*(q(:,1).*q(:,3) -q(:,4).*q(:,2)));
            rpy(:,3) = atan2(2*(q(:,1).*q(:,4) + q(:,2).*q(:,3)), 1 - 2*(q(:,3).^2 + q(:,4).^2));
        end
        
        function mat = quat2mat(q)
            %QUAT2MAT converts quaternion of form qw + qx*i + qy*j + qz*k to 3x3
            %rotation matrix
            qw = q(1); qx = q(2); qy = q(3); qz = q(4);
            mat = ...
                [1 - 2*qy^2 - 2*qz^2,	2*qx*qy - 2*qz*qw,	2*qx*qz + 2*qy*qw;
                2*qx*qy + 2*qz*qw,	1 - 2*qx^2 - 2*qz^2,	2*qy*qz - 2*qx*qw;
                2*qx*qz - 2*qy*qw,	2*qy*qz + 2*qx*qw,	1 - 2*qx^2 - 2*qy^2];
        end
        
        function qinv = quatinv(q)
            %QUATINV Performs Quaternion inverse for q being a nx4 matrix
            %q is of the form: a1 + b1 i + c1 j + d1 k
            qinv = [q(:,1) -q(:,2:4)];
        end
        
        function qres = quatmultiply(q1,q2)
            %Performs Quaternion multiplication of q1*q2. q1 and q2 are nx4 matrices
            %q1 is of the form: a1 + b1 i + c1 j + d1 k
            qres(:,1) = q1(:,1).*q2(:,1) - q1(:,2).*q2(:,2) - q1(:,3).*q2(:,3) - q1(:,4).*q2(:,4);
            qres(:,2) = q1(:,1).*q2(:,2) + q1(:,2).*q2(:,1) + q1(:,3).*q2(:,4) - q1(:,4).*q2(:,3);
            qres(:,3) = q1(:,1).*q2(:,3) - q1(:,2).*q2(:,4) + q1(:,3).*q2(:,1) + q1(:,4).*q2(:,2);
            qres(:,4) = q1(:,1).*q2(:,4) + q1(:,2).*q2(:,3) - q1(:,3).*q2(:,2) + q1(:,4).*q2(:,1);
        end
        
        function vecres = quatrotate(q,vec)
            %QUATROTATE rotate a given vector using a quaternion to provide a rotated point
            sizevec = size(vec,1);%Number of rows
            qvec = [zeros(sizevec,1) vec];
            vecres = QuatLib.quatmultiply(q, ...
                QuatLib.quatmultiply(qvec, QuatLib.quatinv(q)));
            vecres(:,1) = [];
        end        
    end
    
end

