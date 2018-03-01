%Copyright 2018 Joao Bimbo
%Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
%1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
%2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
%THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
classdef Perception < handle
    %PERCEPTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        robot
        prev_robot_state
        particles
        n_particles
        F_max
        K_max
        r_max
        e
        prev_env
        obstacle
        meas_noise
    end
    
    methods
        function obj=Perception(robot,F_max,K_max,r_max,environment,o,noise)
            obj.robot=robot;
            obj.F_max=F_max;
            obj.K_max=K_max;
            obj.r_max=r_max;
            obj.e=environment;
            obj.obstacle=o;
            obj.meas_noise=noise;
        end
        
        function part_out=init_parts(obj,n_parts)
            obj.n_particles=n_parts;
            part_out=zeros(n_parts,6);
            n_l=length(obj.robot.joint_angles);
            %t=floor(n_parts/n_l);
            t(1)=1;
            for i=1:n_l
                t(i+1)=floor(n_parts*obj.robot.link_lengths(i)/sum(obj.robot.link_lengths));
            end
            fm=obj.F_max*2;
            for i=1:n_l
                t_parts=zeros(t(i+1),6);
                t_parts(:,1)=i;
                t_parts(:,2)= rand(t(i+1),1)*obj.robot.link_lengths(i);
                t_parts(:,3)=(rand(t(i+1),1)-0.5)*fm;
                t_parts(:,4)=(rand(t(i+1),1)-0.5)*fm;
                t_parts(:,5)= rand(t(i+1),1)*obj.K_max;
                %     t_parts(:,6)= rand(t(i+1),1)*obj.r_max;
                
                part_out(sum(t(1:i)):sum(t(1:i+1))-1,:)=t_parts;
                i_last=i;
            end
            part_out(sum(t):n_parts,:)=[];
        end
        
        function J=get_jacobian(obj)
            h=0.001;
            for i=1:length(obj.robot.joint_angles)
                [~,T_cur]=obj.robot.fwd();
                next_ang=obj.robot.joint_angles;
                next_ang(i)=next_ang(i)+h;
                [~,T_next]=obj.robot.fwd(next_ang);
                for j=1:size(T_next,3)
                    J(:,i,j)=(obj.robot.T_to_par(T_next(:,:,j))-obj.robot.T_to_par(T_cur(:,:,j)))/h;
                end
            end
        end
        function [j_torque,tor]=calculate_torques(obj,robot,p) %#ok<*INUSL>
            tor=zeros(length(robot.link_lengths),3);
            j_torque=zeros(1,length(robot.link_lengths));
            [~,~,TP,~]=robot.fwd;
            f=[p(3);p(4);0;0];
            r=[0;0;p(2);1];
            
            for i=1:p(1)
                T=TP(:,:,i)\TP(:,:,p(1));
                fi=T*f;
                ri=T*r;
                tor(:,i)=cross(ri(1:3),fi(1:3));
            end
            for i=1:length(robot.joint_angles)
                j_torque(i)=tor(robot.axes(i),i);
            end
        end
        
        function [part_out]=get_likelihoods(obj,particles,aggress)
            %J=obj.get_jacobian;
            part_out=particles;
            for i=1:size(particles,1)
                %if it's not possible that this radius and stiffness
                %generate that force
                %                      if (norm(particles(i,3:4))/particles(i,5)>particles(i,6))
                %                           part_out(i,end)=0;
                %                       else
                t=obj.calculate_torques(obj.robot,particles(i,:));
                
                part_out(i,end)=exp(-aggress*norm(t-(obj.e.reaction_torques.*([1,1,1]+(randn(1,3)*obj.meas_noise)))));
                %                         end
            end
        end
        
        
        
        function parts_out=add_noise(obj,parts_in,perc)
            parts_out=zeros(size(parts_in));
            n=size(parts_in,1);
            if length(perc)==1
                var=perc*[0.5,5.0,5.0,2500,0.025];
            else
                var=perc;
            end
            parts_out(:,1)=parts_in(:,1);
            parts_out(:,2)=parts_in(:,2)+(randn(n,1)*var(1));
            
            %just add random noise
            parts_out(:,3)=parts_in(:,3)+(randn(n,1)*var(2));
            parts_out(:,4)=parts_in(:,4)+(randn(n,1)*var(2));
            parts_out(:,5)=parts_in(:,5)+(randn(n,1)*var(4));
            %      parts_out(:,6)=parts_in(:,6)+(randn(n,1)*var(5));
            parts_out=obj.limit_parts(parts_out);
        end
        function parts_out=limit_parts(obj,parts_in)
            parts_out=parts_in;
            idx=find(parts_out(:,2)<=0);
            parts_out(idx,2)=rand(length(idx),1)*max(parts_out(:,2));
            idx=find(parts_out(:,5)<=0);
            parts_out(idx,5)=rand(length(idx),1)*obj.K_max;%   max(parts_out(:,5));
            %parts_out(parts_out(:,6)<=0,6)=rand*median(parts_out(:,6));
            for i=1:length(obj.robot.link_lengths)
                idx=find(parts_out(:,1)==i & parts_out(:,2)>obj.robot.link_lengths(i));
                parts_out(idx,2) = rand(length(idx),1)*obj.robot.link_lengths(i);
            end
        end
        
        function save_robot_state(obj)
            obj.prev_robot_state=obj.robot.copy;
            obj.prev_env=obj.e.copy;
        end
        
        function [part_out]=motion_model2(obj,particles)
            part_out=zeros(size(particles));
            for i=1:size(particles,1)
                part_out(i,1)=particles(i,1);
                
                x0=obj.prev_robot_state.base_pos(particles(i,1),:)+...
                    particles(i,2)*obj.prev_robot_state.link_directions(particles(i,1),:);
                [v,d,l2]=obj.calc_next_l( ...
                    obj.robot.base_pos(particles(i,1),:), ...
                    obj.robot.base_pos(particles(i,1)+1,:), ...
                    x0);
                part_out(i,2)=l2;
                f_new=obj.calc_next_f(particles(i,:),v,d);
                part_out(i,3:4)=f_new(1:2);
                part_out(i,5)=particles(i,5);
            end
        end
        
        function [part_out]=motion_model(obj,particles)
            part_out=zeros(size(particles));
            for i=1:size(particles,1)
                part_out(i,1)=particles(i,1);
                l_new=particles(i,2)+(obj.e.collision_l-obj.prev_env.collision_l);
                part_out(i,2)=l_new;
                %f_new=obj.f_model(particles(i,:),obj.prev_robot_state,obj.robot,l_new);
                f_diff_cheat= ...
                    obj.prev_robot_state.get_force_in_link_frame(obj.prev_env.links_in_collision(1),obj.prev_env.collision_force) ...
                    -obj.robot.get_force_in_link_frame(obj.e.links_in_collision(1),obj.e.collision_force);
                
                f_new=particles(i,3:4)+(f_diff_cheat(1:2)'/obj.obstacle.stiffness)*particles(i,5);
                
                part_out(i,3:4)=f_new(1:2);
                part_out(i,5)=particles(i,5);
            end
        end
        
        function f_new=f_model(obj,part,prev_state,cur_state,l_new)
            link=part(1,1);
            l_old=part(1,2);
            f_old=[part(1,3:4),0];
            k=part(5);
            
            prev_contact=prev_state.get_contact_location(link,l_old);
            new_contact=cur_state.get_contact_location(link,l_new);
            prev_force=prev_state.get_contact_force(link,f_old);
            dx_contact=new_contact-prev_contact;
            dx_l_frame=cur_state.get_force_in_link_frame(link,dx_contact);
            fn=(prev_force/norm(prev_force));
            
            indent=dot(dx_l_frame,fn)
            f_new=f_old-k*indent;
            [f_old;dx_contact';dx_l_frame';f_new;-cur_state.get_force_in_link_frame(obj.e.links_in_collision(1),obj.e.collision_force)']
            
        end
        
        
        function f_new=calc_next_f(obj,part_in,v,d)
            f_old=[part_in(3:4),0];
            k=part_in(5);
            fn=(f_old/norm(f_old));
            indent=dot(v,fn)*fn;
            
            f_new=f_old-k*indent;
        end
        function [v,d,l2]=calc_next_l(obj,x1,x2,x0)
            %x1: base
            %x2: tip
            %x3: x0 prev_point
            %d=norm(cross(x0-x1,x0-x2))/norm(x2-x1);
            
            %previous model
            t=-dot(x1-x0,x2-x1)/norm(x2-x1)^2;
            p=x1+(x2-x1)*t;
            l2=norm(x2-x1)*t;
            v=p-x0;
            d=norm(v);
            %simpler model
        end
        
        function [part_out]=motion_model_det(obj,particles)
            part_out=zeros(size(particles));
            [~,~,T1b,~]=obj.prev_robot_state.fwd;
            [~,~,T2b,~]=obj.robot.fwd;
            f2_cheated=obj.e.force_in_link_frame/norm(obj.e.force_in_link_frame);
            r_cheated=obj.obstacle.size;
            k_cheated=obj.obstacle.stiffness;
            
            for i=1:size(particles,1)
                T1=T1b(:,:,particles(i,1));
                T2=T2b(:,:,particles(i,1));
                l1=particles(i,2);
                f1=[particles(i,3:4),0];
                fn1=f1/norm(f1);
                f1x=fn1(1);
                f1y=fn1(2);
                nf=norm(f1);
                k=particles(i,5);
                
                
                %M=T2\T1*([0,0,l1,0]'-[fn1,0]'*(r_cheated-nf/k))-T2\([T2(1:3,4)-T1(1:3,4);0]);
                %fn2=M(1)*k/f2_cheated(1)+r_cheated*k;
                
                r_cheated=particles(i,6);
                %M=T2\T1*([0,0,l1,0]'-[fn1,0]'*(r_cheated-nf/k))-T2\([T2(1:3,4)-T1(1:3,4);0]);
                
                M=T2\([T1(1:3,4)-T2(1:3,4);0])+T2\T1*([0,0,l1,0]'-[fn1,0]'*(r_cheated-nf/k));
                syms f2x f2y nf2
                fff=[f2x*r_cheated-f2x*nf2/k==M(1);f2y*r_cheated-f2y*nf2/k==M(2);f2x*f2x+f2y*f2y==1];
                sol=solve(fff);
                
                f2_cheated_c=f2_cheated;
                fn2=double(sol.nf2(1));
                f2_cheated(1)=double(sol.f2x(1));
                f2_cheated(2)=double(sol.f2x(1));
                %fn2=M(1)*k/f2_cheated(1)+r_cheated*k
                [f2_cheated_c;f2_cheated]
                
                
                part_out(i,1)=particles(i,1);
                part_out(i,2)=M(3);
                part_out(i,3)=f2_cheated(1)*fn2;
                part_out(i,4)=f2_cheated(2)*fn2;
                part_out(i,5)=particles(i,5);
            end
        end
        
        function [part_out]=motion_model_rand(obj,particles)
            part_out=zeros(size(particles));
            [~,~,T1b,~]=obj.prev_robot_state.fwd;
            [~,~,T2b,~]=obj.robot.fwd;
            %f2_cheated=obj.e.force_in_link_frame/norm(obj.e.force_in_link_frame);
            
            for i=1:size(particles,1)
                %r=0.04+rand*0.02;
                %r=0.05;
                T1=T1b(:,:,particles(i,1));
                T2=T2b(:,:,particles(i,1));
                l1=particles(i,2);
                f1=[particles(i,3:4),0];
                fn1=f1/norm(f1);
                nf=norm(f1);
                k=particles(i,5);
                r=particles(i,6);
                %Use Cramer's rule: https://math.stackexchange.com/questions/406864/intersection-of-two-lines-in-vector-form
                % M=T2\([T1(1:3,4)-T2(1:3,4);1])+T2\T1*([0,0,l1,0]'-[fn1,0]'*(r-nf/k))

                %sphere centre (confirmed):
                M=T2\([T1(1:3,4)-T2(1:3,4);0])+T2\T1*([0,0,l1,0]'-[fn1,0]'*(r-nf/k)) ;
                f2n=-M(1:2)/norm(M(1:2));
                fn2=(M(1)/f2n(1)+r)*k;
                
                
                part_out(i,1)=particles(i,1);
                part_out(i,2)=M(3);
                part_out(i,3)=f2n(1)*fn2;
                part_out(i,4)=f2n(2)*fn2;
                part_out(i,5)=particles(i,5);
                part_out(i,6)=particles(i,6);
                
            end
        end
        function [part_out]=motion_model_proper(obj,particles,var)
            sz=size(particles,1);
            part_out=zeros(sz,6);
            [~,~,T1b,~]=obj.prev_robot_state.fwd;
            [~,~,T2b,~]=obj.robot.fwd;
            n=particles(:,1);
            l1=particles(:,2);
            k=particles(:,5);
            weight=particles(:,end);            
            f1=[particles(:,3:4)];
            nf=sqrt(sum(f1.^2,2));
            fn1=f1./[nf,nf];            
            std_dev=var;      
            prob=ones(size(particles,1),1);   
            ra=rand(sz(1),1);
            LAi = log10(0.0101); LB = log10(10.0110);
            r=10.^(LAi + (LB-LAi) * ra);
            for i=1:sz
                T1=T1b(:,:,n(i));
                T2=T2b(:,:,n(i));
                
                ct1=T2\T1(:,4)+T2\T1*[0,0,l1(i),0]'; %c(t-1)
                r_min=norm(ct1(1:2));
                %sphere centre (confirmed):
                d1=(r(i)-nf(i)/k(i));
                M=T2\([T1(1:3,4)-T2(1:3,4);0])+T2\T1*([0,0,l1(i),0]'-[fn1(i,:),0,0]'*d1) ;
                f2n=-M(1:2)/norm(M(1:2));
                fn2=(M(1)/f2n(1)+r(i))*k(i);
                part_out(i,:)=[n(i),M(3),f2n(1)*fn2,f2n(2)*fn2,k(i),weight(i)*prob(i)];
                if(d1<0)
                    part_out(i,:)=particles(i,:);
                end
            end
            
        end
        
        function part_out=create_part(obj,part_in)
            part_out=part_in;
        end
        function parts_out=resample(obj,parts_in,j)
            sz=size(parts_in);
            s=sum(parts_in(:,end));
            parts_out=zeros(floor(sz(1)*j),sz(2));
            eta=0;
            for i=1:size(parts_out,1)
                r=rand*s;
                st=0;
                for j=1:size(parts_in,1)
                    st=st+parts_in(j,end);
                    if(st>r)
                        parts_out(i,:)=obj.create_part(parts_in(j,:));
                        eta=eta+parts_out(i,end);
                        break;
                    end
                end
            end
            parts_out(:,end)=parts_out(:,end)/eta;
        end
        
        function [f]=force_from_torques(obj,link_nr,torques,l)
            cl=obj.robot.base_pos(link_nr,:)'+l*obj.robot.link_direction(link_nr);
          [~,~,Tb,~]=obj.robot.fwd;
            for i=1:link_nr
                rv(:,i)=cl-obj.robot.base_pos(i,:)';
                %rv(:,i)=cl;
                T1r=Tb(:,:,i)\[rv(:,i);0];%T1r=inv(Tb(:,:,i))*[rv(:,i);0];
                T1rx=skew(T1r(1:3))*inv(Tb(1:3,1:3,i));
                A(i,:)=T1rx(obj.robot.axes(i),:);
                b(i)=torques(i);%tau_t(r.axes(i));
            end
            A(end+1,:)=obj.robot.link_direction(link_nr);
            b(end+1)=0;
            f_w=A\b'; %f_w=inv(A)*b';
            f=Tb(:,:,link_nr)\[f_w;0];
            f=f(1:2);
        end
        
        function parts=init_parts_heur(obj,n,link_nr)
            parts=zeros(n,6);
            parts(:,1)=link_nr;
            parts(:,2)=rand(n,1)*obj.robot.link_lengths(link_nr);
            parts(:,5)=rand(n,1)*obj.K_max;
            for i=1:n
                parts(i,3:4)=obj.force_from_torques(link_nr,obj.e.reaction_torques,parts(i,2));
            end
        end
        
    end
end