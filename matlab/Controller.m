%Copyright 2018 Joao Bimbo
%Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
%1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
%2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
%THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
classdef Controller < handle
    %CONTROLLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Kp
        Ki
        Kd
        robot
        target
        cur_controller
    end
    
    methods
        function obj=Controller(robot,p,i,d)
            obj.robot=robot;
            obj.cur_controller='position';
            obj.Kp=p;
            obj.Ki=i;
            obj.Kd=d;
        end
        
        function obj=control_step(obj)
            if strcmp(obj.cur_controller,'position')
                obj.control_pos;
            elseif strcmp(obj.cur_controller,'torque')
                return
            elseif strcmp(obj.cur_controller,'impedance')
                obj.control_imp;
                return
            else
                return
            end
            
        end
        
        function obj=control_imp(obj)           
           obj.robot.commanded_torque=obj.Kp.*(obj.target-obj.robot.joint_angles);           
        end
        
        function obj=control_pos(obj)
            x_err=obj.target-obj.fwd(obj.robot.joint_angles);
            J=obj.jac(obj.robot.joint_angles);
            dtheta=obj.Kp'.*(pinv(J)*x_err');
            obj.robot.commanded_torque=dtheta;
        end
        
        function T=fwd_kin(obj,angles)
            T=eye(3);
            for i=1:length(angles)
                R=obj.rot_m(angles(i));
                T=T*[R,R*[obj.robot.link_lengths(i);0];[0,0,1]];
            end
            return
        end
        function x=fwd(obj,angles)
            T=obj.fwd_kin(angles);
            x=[T(1,end),T(2,end),atan2(T(2,1),T(1,1))];
            return
        end
        
        function J=jac(obj,angles)
            x0=obj.fwd(angles);
            J=zeros(length(angles),length(x0));
            h=0.001;
            for j=1:length(angles)
                angles_tmp=angles;
                angles_tmp(j)=angles_tmp(j)+h;
                x=obj.fwd(angles_tmp);
                J(:,j)=x-x0;
            end
        end
        function R=rot_m(obj,ang)
            R=[cos(ang), -sin(ang);
                sin(ang), cos(ang)];
            return
        end
        
    end
    
end

