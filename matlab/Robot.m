%Copyright 2018 Joao Bimbo
%Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
%1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
%2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
%THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
classdef Robot < handle
    %ROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        joint_angles
        link_lengths
        base_pos
        commanded_torque
        ee_pos
        axes
        link_directions
    end
    
    methods
        function obj=Robot(l,angs)
            if(length(l) ~= length(angs))
                error('links and angles have different lengths')
            end
            obj.link_lengths=l;
            obj.joint_angles=angs;
            obj.commanded_torque=zeros(size(obj.joint_angles));
            obj.axes=[3,1,1];
            obj.fwd;
        end
        function obj=set_angle(obj,n,ang)
            obj.joint_angles(n)=ang;
            % set(obj,'link_angle',ang)
            obj.fwd;
            return
        end
        function R=rot_z(obj,ang)
            R=[cos(ang), -sin(ang) 0 ;
                sin(ang), cos(ang) 0 ;
                0 0 1];
            return
        end
        function R=rot_x(obj,ang)
            R=[1 0 0 ;
                0 cos(ang), -sin(ang) ;
                0 sin(ang), cos(ang)];
            return
        end
        
        function obj=add_link(obj,len,ang)
            obj.joint_angles=[obj.joint_angles,ang];
            obj.link_lengths=[obj.link_lengths,len];
            obj.fwd;
        end
        
        function [ee_pos,Ts,Tb,Tp]=fwd(obj,angs)
            if(nargin==1)
                angles=obj.joint_angles;
            else
                angles=angs;
            end
            obj.base_pos(1,:)=[0,0,0];
            %   bp=[obj.base_pos(1,:),1];
            T=eye(4);
            if(obj.axes(1)==1)
                R=obj.rot_x(angles(1));
            elseif(obj.axes(1)==3)
                R=obj.rot_z(angles(1));
            else
                R=eye(3);
            end
            Tp(:,:,1)=[R,R*[0.26,0,obj.link_lengths(1)]';0,0,0,1];
            %            Tp(:,:,1)=[R,R*[ 0; 0; 0];[0,0,0,1]];
            %            T=T*[R,R*[ 0; 0; obj.link_lengths(1)];[0,0,0,1]];
            Ts(:,:,1)=[R,[0,0,obj.link_lengths(1)]';0,0,0,1];
            Tb(:,:,1)=[Tp(:,1:3),[0,0,0,1]'];
            T=T*Tp(:,:,1);
            for i=2:length(angles)
                if(obj.axes(i)==1)
                    R=obj.rot_x(angles(i));
                elseif(obj.axes(i)==3)
                    R=obj.rot_z(angles(i));
                else
                    R=eye(3);
                end
                
                %                prev_T=T;
                Tp(:,:,i)=[R,R*[0.0;0;obj.link_lengths(i)];[0,0,0,1]];
                % Tb(:,:,i)=Tb(:,:,i-1)*Tp(:,:,i-1)*[R,[0,0,0]';[0,0,0,1]];
                T=T*Tp(:,:,i);
                Tb(:,:,i)=T*[eye(3),[0,0,-obj.link_lengths(i)]';[0,0,0,1]];
                %T=T*[R,R*[0.04;0;obj.link_lengths(i)];[0,0,0,1]];
                
                %  Tb(:,:,i)=[T(:,1:3),[prev_T(:,4)+[0.04,0,0.0,0]']];
                Ts(:,:,i)=Tb(:,:,i)*[eye(3),[0,0,obj.link_lengths(i)]';[0,0,0,1]];
                obj.base_pos(i,:)=Tb(1:3,4,i);
            end
            obj.base_pos(i+1,:)=Ts(1:3,4,end);
            
            obj.ee_pos=obj.T_to_par(Ts(:,:,end));
            ee_pos=obj.ee_pos;
            
            return
        end
        function dirs=get_directions(obj)
            for i=1:length(obj.joint_angles)
                obj.link_directions(i,:)=obj.link_direction(i);
            end
            dirs=obj.link_directions;
        end
        function ee_pos=T_to_par(obj,T)
            if(license('test','Aerospace_Toolbox'))
                [ay,ap,ar]=dcm2angle(T(1:3,1:3));
            else
                rpy=DCM2RPY(T(1:3,1:3));
                ar=rpy(1);
                ap=rpy(2);
                ay=rpy(3);
            end
            ee_pos=[T(1,end),T(2,end),T(3,end),ar,ap,ay];
            
        end
        function v=link_direction(obj,link_nr)
            [~,Tb,Ts,~]=obj.fwd;
            p1=Ts(1:3,4,link_nr);
            p2=Tb(1:3,4,link_nr);
            v=(p2-p1);v=v/norm(v);
        end
        function v2=surf_normal(obj,link_nr,l)
            v=obj.link_direction(link_nr);
            v2=[v(2),-v(1)];
        end
        % Copy function - replacement for matlab.mixin.Copyable.copy() to create object copies
        function newObj = copy(obj)
            try
                % R2010b or newer - directly in memory (faster)
                objByteArray = getByteStreamFromArray(obj);
                newObj = getArrayFromByteStream(objByteArray);
            catch
                % R2010a or earlier - serialize via temp file (slower)
                fname = [tempname '.mat'];
                save(fname, 'obj');
                newObj = load(fname);
                newObj = newObj.obj;
                delete(fname);
            end
        end
        
        function p=get_contact_location(obj,link_nr,length)
            [~,~,T]=obj.fwd;
            p=T(:,:,link_nr)*[0,0,length,1]';
            p=p(1:3);
            
        end
        
        function p=get_contact_force(obj,link_nr,force)
            [~,~,T]=obj.fwd;
            p=T(:,:,link_nr)*[force(1),force(2),force(3),0]';
            p=-p(1:3);
        end
        
        function p=get_force_in_link_frame(obj,link_nr,force)
            [~,~,T]=obj.fwd;
            p=T(:,:,link_nr)\[force(1),force(2),force(3),0]';
            p=-p(1:3);
        end
        
        
        
    end
end
