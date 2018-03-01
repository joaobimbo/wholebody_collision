%Copyright 2018 Joao Bimbo
%Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
%1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
%2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
%THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
classdef Visualizer < handle
    %VISUALIZER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        environm
        fig_robot
        fig_robot_big
        controller
        log
        fig_link
    end
    
    methods
        function obj=Visualizer(fh,envir,controller)
            obj.fig_robot=fh;
            obj.fig_robot_big=fh;
            axes(obj.fig_robot)
            view(100,-30)
            obj.environm=envir;
            obj.controller=controller;
        end
        function draw(obj)
            % figure(obj.fig_robot);
            axes(obj.fig_robot)
            [az,el]=view;
            cla
            [~,Ts,Tb,~]=obj.environm.robot.fwd;
            obj.environm.collision;
            hold on;
            
            for i=1:size(Ts,3)
                line([Tb(1,4,i), Ts(1,4,i)], ...
                    [Tb(2,4,i), Ts(2,4,i)], ...
                    [Tb(3,4,i), Ts(3,4,i)], ...
                    'LineWidth',10);
            end
            %plot(obj.environm.obstacle.position(1),obj.environm.obstacle.position(2),'k.','MarkerSize',3);
            %plot(obj.environm.obstacle.position(1),obj.environm.obstacle.position(2),'ko','MarkerSize',20);
            
            if strcmp(obj.environm.obstacle.type,'sphere')
            obj.drawcircle(obj.environm.obstacle.position(1),obj.environm.obstacle.position(2),...
                obj.environm.obstacle.position(3),obj.environm.obstacle.size,'g.')
            elseif strcmp(obj.environm.obstacle.type,'edge')
                obj.drawwall(obj.environm.obstacle,'g.')
            end
            
            for i=1:size(obj.environm.robot.base_pos,1)
                plot3(obj.environm.robot.base_pos(i,1),obj.environm.robot.base_pos(i,2),obj.environm.robot.base_pos(i,3),'bo','MarkerSize',10);
            end
            
            plot3(obj.environm.nearest_location(:,1),obj.environm.nearest_location(:,2),obj.environm.nearest_location(:,3),'y.')
            if ~isempty(obj.environm.collision_location)
                plot3(obj.environm.collision_location(:,1),obj.environm.collision_location(:,2),obj.environm.collision_location(:,3),'r.')
                
                scale=0.03;
                for i=1:size(obj.environm.collision_location,1)
                    quiver3(obj.environm.collision_location(i,1),obj.environm.collision_location(i,2),obj.environm.collision_location(i,3), ...
                        scale*obj.environm.collision_force(i,1),scale*obj.environm.collision_force(i,2),scale*obj.environm.collision_force(i,3),'r','LineWidth',2,'MaxHeadSize', 0.1/norm(scale*obj.environm.collision_force(i,1)))
                end
                
            end
            
            
            axis([-sum(obj.environm.robot.link_lengths), ...
                sum(obj.environm.robot.link_lengths), ...
                -sum(obj.environm.robot.link_lengths), ...
                sum(obj.environm.robot.link_lengths), ...
                -sum(obj.environm.robot.link_lengths), ...
                sum(obj.environm.robot.link_lengths)])
            daspect([1,1,1])
            view(az,el)
            
            
            obj.draw_frames(obj.fig_robot,Tb);
            baseT=eye(4);
            baseT(3,4)=-0.1;
            obj.draw_frames(obj.fig_robot,baseT);
            
            return
        end
        
        function drawwall(obj,obstacle,opts)    
            p=zeros(400,4);
            k=1;
            l=20;
            %for i=0:l-1
            for i=linspace(0,1,l)
                p(k:k+l-1,:)=[linspace(0,1,l)',zeros(l,1),ones(l,1)*i,ones(l,1)];
                k=k+l;
                p(k:k+l-1,:)=[zeros(l,1),linspace(0,1,l)',ones(l,1)*i,ones(l,1)];
                k=k+l;
            end
            p=[obstacle.edge*p']';
            %plot3(obstacle.edge(1,4),obstacle.edge(2,4),obstacle.edge(3,4),opts)
            plot3(p(:,1),p(:,2),p(:,3),opts)
        end
        
        function drawcircle(obj,x,y,z,r,opt)
            %x and y are the coordinates of the center of the circle
            %r is the radius of the circle
            %0.01 is the angle step, bigger values will draw the circle faster but
            %you might notice imperfections (not very smooth)
            %            ang=0:0.01:2*pi;
            %            xp=r*cos(ang);
            %            yp=r*sin(ang);
            %            plot(x+xp,y+yp,opt);
            
            phi=linspace(0,pi,20);
            theta=linspace(0,2*pi,20);
            [phi,theta]=meshgrid(phi,theta);
            
            xp=r*sin(phi).*cos(theta);
            yp=r*sin(phi).*sin(theta);
            zp=r*cos(phi);
            plot3(x+xp,y+yp,z+zp,opt);
            
        end
        
        function sim_to_steady(obj)
            t=1;
            obj.log=[];
            obj.controller.control_step;
            obj.environm.sim_step;
            while(norm(obj.environm.t_torque)>1e-3)
                %norm(obj.environm.t_torque)
                obj.controller.control_step;
                obj.environm.sim_step;
                t=t+1;
                if(mod(t,1001)==1000)
                    obj.draw;
                    drawnow
                    %commandwindow
                end
                obj.log(t,1:6)=obj.environm.robot.ee_pos;
                
            end
            obj.draw;
            %     commandwindow
            
        end
        function obj=switch_controller(obj,controller)
            obj.controller=controller;
        end
        
        function draw_frames(obj,fig,T)
            hold on
            for i=1:size(T,3)
                quiver3(T(1,4,i),T(2,4,i),T(3,4,i),T(1,1,i)*0.1,T(2,1,i)*0.1,T(3,1,i)*0.1,'r')
                quiver3(T(1,4,i),T(2,4,i),T(3,4,i),T(1,2,i)*0.1,T(2,2,i)*0.1,T(3,2,i)*0.1,'g')
                quiver3(T(1,4,i),T(2,4,i),T(3,4,i),T(1,3,i)*0.1,T(2,3,i)*0.1,T(3,3,i)*0.1,'b')
            end
        end
        
        
        function draw_link_forces(obj,i,particles)
            if(exist('obj.fig_link','var')==0)
                obj.fig_robot=axes;
            end
            axes(obj.fig_robot)
            
            hold on
            line([obj.environm.robot.base_pos(i,1), ...
                obj.environm.robot.base_pos(i+1,1)], ...
                [obj.environm.robot.base_pos(i,2), ...
                obj.environm.robot.base_pos(i+1,2)], ...
                'LineWidth',10);
            scale=2;
            plot(particles(:,5),particles(:,6),'r.');
            quiver(particles(:,5)-scale*particles(:,2), ...
                particles(:,6)-scale*particles(:,3), ...
                scale*particles(:,2), ...
                scale*particles(:,3))
            
            daspect([1,1,1])
            
            
        end
    end
end



