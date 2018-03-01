%Copyright 2018 Joao Bimbo
%Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
%1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
%2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
%THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
classdef Tools
    
    methods(Static)
        function p=get_contact_location(robot,link_nr,length)
            [~,~,T]=robot.fwd;
            p=T(:,:,link_nr)*[0,0,length,1]';
            p=p(1:3);
            
        end
        
        function p=get_contact_force(robot,link_nr,force)
            [~,~,T]=robot.fwd;
            p=T(:,:,link_nr)*[force(1),force(2),0,0]';
            p=-p(1:3);
        end
        
        function [t,t_total]=get_part_torques(robot,env,part,n)
            dofs=length(robot.joint_angles);
            t=zeros(1,dofs);
            t_total=zeros(dofs,3);
            F=Tools.get_contact_force(robot,part(n,1),part(n,3:4))
            pc=Tools.get_contact_location(robot,part(n,1),part(n,2));
            for i=1:dofs
                pb=Tools.get_contact_location(robot,i,0);
                tt=env.calc_torques(F,pc,pb);
                t_total(i,:)=tt;
                t(1,i)=tt(robot.axes(i));
                
            end
        end
        
        function draw_force(fig,robot,part)
            axes(fig);
            scale=0.03;
            for n=1:size(part,1)
                F=Tools.get_contact_force(robot,part(n,1),part(n,3:4));
                pc=Tools.get_contact_location(robot,part(n,1),part(n,2));
                quiver3(pc(1),pc(2),pc(3),-scale*F(1),-scale*F(2),-scale*F(3),'b:','LineWidth',2,'MaxHeadSize', 0.5/norm(scale*F))
                
            end
        end
        
        function p_true=get_true_part(robot,e,o,p)
            [~,~,TT]=robot.fwd;
            Fj=inv(TT(1:3,1:3,e.links_in_collision(1)))*[e.collision_force]';
            l=e.collision_l;
            p_true=[e.links_in_collision(1),l,Fj(1),Fj(2),o.stiffness,0];
            p_true=p.get_likelihoods(p_true,1);
            
        end
        
        function res=check_motion_model(p,o)
            [~,~,TT]=p.prev_robot_state.fwd;
            Fj=inv(TT(1:3,1:3,p.prev_env.links_in_collision(1)))*[p.prev_env.collision_force]';
            l=p.prev_env.collision_l;
            p_prev_true=[p.e.links_in_collision(1),l,Fj(1),Fj(2),o.stiffness,o.size,0];
            
            [~,~,TT]=p.robot.fwd;
            Fj=inv(TT(1:3,1:3,p.e.links_in_collision(1)))*[p.e.collision_force]';
            l=p.e.collision_l;
            p_true=[p.e.links_in_collision(1),l,Fj(1),Fj(2),o.stiffness,o.size,0];
            
            p_fores=p.motion_model_proper(p_prev_true);
            
        %    p_false=p_prev_true;
        %    p_false(5)=p_prev_true(5)*2;
        %    p_fores_false=p.motion_model2(p_false);
            
            
            
            res=[p_prev_true;p_fores;p_true];
            res=p.get_likelihoods(res,1);
        end
        function write_2_pics_to_file(filename,f1,f2)
                    set(f1,'Position',[0,100,800,600])
        %set(g.vis.fig_robot,'Position',[00,500,600,400],'Units','pixels')
        set(f1,'Color','w')
        frame = getframe(f1);
        im = frame2im(frame);
        frame2 = getframe(f2);
        im=[im,frame2im(frame2)];
        [imind,cm] = rgb2ind(im,256);
        imwrite(imind,cm,filename,'png');
        %imwrite(imind,cm,filename,'gif','WriteMode','append');

        end
    end
end