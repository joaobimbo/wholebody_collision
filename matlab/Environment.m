%Copyright 2018 Joao Bimbo
%Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
%1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
%2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
%THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
classdef Environment <handle
    %ENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        robot
        obstacle
        collision_location
        collision_distance
        collision_l
        nearest_location
        collision_force
        reaction_torques
        links_in_collision;
        t_torque
        force_in_link_frame
    end
    methods
        function obj=Environment(robot,obstacle)
            obj.robot=robot;
            obj.obstacle=obstacle;
            obj.collision;
        end
        
        function obj=collision(obj)
            obj.collision_location=[];
            obj.collision_force=[];
            obj.links_in_collision=[];
            obj.collision_l=[];
            if strcmp(obj.obstacle.type,'sphere')
                for i=1:length(obj.robot.base_pos)-1
                    [obj.collision_distance(i,:),obj.nearest_location(i,:)] ...
                        =obj.distance(obj.robot.base_pos(i,1:3), ...
                        obj.robot.base_pos(i+1,1:3),obj.obstacle.position);
                    if obj.collision_distance(i,:)<obj.obstacle.size
                        obj.collision_location=[obj.collision_location;...
                            obj.nearest_location(i,:)];
                        
                        nF=obj.nearest_location(i,:)-obj.obstacle.position;
                        nF=nF/norm(nF);
                        dx=obj.obstacle.size-obj.collision_distance(i,:);
                        
                        obj.collision_force=[obj.collision_force;...
                            obj.obstacle.stiffness*dx*nF];
                        
                        obj.links_in_collision=[obj.links_in_collision;i];
                        obj.collision_l=[obj.collision_l;norm(obj.nearest_location(i,:)-obj.robot.base_pos(i,:))];
                        
                    end
                end
            elseif strcmp(obj.obstacle.type,'edge')
                for i=1:length(obj.robot.base_pos)-1
                    iT=inv(obj.obstacle.edge);
                    link_direction=obj.robot.link_direction(i);
                    link_length=obj.robot.link_lengths(i);
                    link_direction_l=iT*[link_direction;0];
                    base_l=iT*[obj.robot.base_pos(i,:),1]';
                    verts=obj.edge_intersections(base_l,link_direction_l,link_length);
                    if(size(verts,1)>0)
                        obj.links_in_collision=[obj.links_in_collision;i];
                        centroid=sum(verts(1:2,:)')/3;          
                        link_normal2d=[0,-1;1,0]*link_direction_l(1:2)/norm(link_direction_l(1:2));
                        lg=[link_direction_l(1) -link_direction_l(2);link_direction_l(2),link_direction_l(1)]\[centroid(1:2)'-base_l(1:2)];
                        cl=base_l+lg(1)*link_direction_l;
                        link_normal_l=[link_normal2d;0;0];
                        
                        
                        dd=-cl(1:2)./link_normal_l(1:2);
                        
                        %Distance to origin: http://mathworld.wolfram.com/Line-LineDistance.html
                        %dd=(abs(verts(2,1)*(verts(1,1) - verts(1,2)) - verts(1,1)*(verts(2,1) - verts(2,2)))^2)^(1/2)/(abs(verts(1,1) - verts(1,2))^2 + abs(verts(2,1) - verts(2,2))^2)^(1/2);                                                
                                               
                        dx=min(dd)*link_normal_l;
                        coll_w=obj.obstacle.edge*cl;
                        obj.collision_l=[obj.collision_l,lg(1)];
                        obj.collision_location=[obj.collision_location;...
                            coll_w(1:3)'];
                        
                        force_w=obj.obstacle.stiffness*(obj.obstacle.edge*dx);
                        
                        obj.collision_force=[obj.collision_force;...                            
                            force_w(1:3)'];    
                        
                       obj.nearest_location(i,:) = coll_w(1:3)';
                    end
                    obj.nearest_location(i,:)=[0,0,0];
                end
            end
            obj.calculate_reaction_torques;
        end
        
        function inters=edge_intersections(obj,base_l,link_direction_l,link_length)
            ab=-base_l(1:2)./link_direction_l(1:2);
            inters=[];
            i_in=base_l+ab(1)*link_direction_l;
            i_out=base_l+ab(2)*link_direction_l;
            if(ab<link_length && ab(1)>-1e-5 && ab(2)>-1e-5 && ...                
                        i_in(1)>=-1e-5 && i_in(2)>=-1e-5 && ...
                        i_out(1)>=-1e-5 && i_out(1)>=-1e-5)
                inters=[i_in, i_out];
            end
        end
        
        function obj=calculate_reaction_torques(obj)
            obj.reaction_torques=zeros(1,length(obj.robot.link_lengths));
            [~,~,Ts]=obj.robot.fwd;
            obj.force_in_link_frame=[];
            for i=1:length(obj.links_in_collision)
                obj.force_in_link_frame(i,:)=inv(Ts(1:3,1:3,obj.links_in_collision(i)))*obj.collision_force(i,:)';
                for j=1:obj.links_in_collision(i)
                    t_jb=obj.calc_torques(obj.collision_force(i,:),obj.collision_location(i,:),obj.robot.base_pos(j,1:3));
                    t_j=dot(t_jb,Ts(1:3,obj.robot.axes(j),j));
                    obj.reaction_torques(j)= obj.reaction_torques(j) + t_j;
                end
            end
        end
        
        function [d,c,cp_obs]=distance_lines(obj,link_b,link_dir,link_size,obst)
            % DISTANCE  Distance between link and obstacle
            %   [d,c]=distance(obj,link_b,link_t,obst)
            %
            edge_dir=[0,0,1]';
            edge_pos=[0,0,0]';
            
            a=link_dir'*link_dir;
            b=link_dir'*edge_dir;
            c=1;
            w0=link_b-edge_pos;
            d=link_dir'*w0;
            e=[0,0,1]*w0;
            acb=a*c-(b*b);
            if acb==0
                print('error')
            else
                sc=(b*e-c*d)/acb;
                tc=(a*e-b*d)/acb;
            end
            if sc<0
                c=link_b;
            elseif sc>link_size
                c=link_b+link_dir*link_size;
            else
                c=link_b+link_dir*sc;
            end
            d=norm(c-(edge_pos+tc*edge_dir));
            cp_obs=edge_pos+tc*edge_dir;
        end
        
        function [d,c]=distance(obj,link_b,link_t,obst)
            % DISTANCE  Distance between link and obstacle
            %   [d,c]=distance(obj,link_b,link_t,obst)
            %
            x1=link_b;
            x2=link_t;
            x0=obst;
            
            if(norm(x2-x1)==0)
                c=x2;
                d=norm(x0-x1);
                disp('uu')
            else
                t=-(dot((x2-x1),(x1-x0)))/dot((x2-x1),(x2-x1));
                if t>1
                    c=x2;
                    d=norm(x2-x0);
                elseif t<0
                    c=x1;
                    d=norm(x1-x0);
                else
                    d=norm(cross((x2-x1),(x0-x2)))/norm(x2-x1);
                    c=x1+(x2-x1)*t;
                end
            end
            %    pause
        end
        function t=calc_torques(obj,F,contact_p,joint_p)
            %function t=calc_torques(obj,F,contact_p,joint_p)            
            r=(contact_p-joint_p);
            %t=(r(1)*F(2))-(r(2)*F(1));  %2-D
            t=cross(r,F);
        end
        
        function sim_step(obj)
            step=0.0001;
            obj.t_torque=zeros(size(obj.robot.joint_angles));
            obj.robot.fwd;
            obj.collision;
            obj.t_torque=obj.robot.commanded_torque+obj.reaction_torques;
            obj.robot.joint_angles=obj.robot.joint_angles+step*obj.t_torque;
            %for i=1:length(obj.robot.joint_angles)
            %    obj.t_torque(i)=obj.robot.commanded_torque(i)+obj.reaction_torques(i);
            %    obj.robot.set_angle(i,obj.robot.joint_angles(i)+step*obj.t_torque(i));
            %end
        end
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
    end
    
end

