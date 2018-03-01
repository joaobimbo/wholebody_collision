parts=parts_saved;
r.joint_angles=des_angs;
c.target=des_angs;
g.vis.sim_to_steady
p_true=Tools.get_true_part(r,e,o,p);
[parts]=p.get_likelihoods(parts,1); %p(z|x)
if DRAW
    plot_particles([fg,fg2],p,parts,p_true);
    axes(g.vis.fig_robot);view(60,20);xlabel('$x$ [m]');ylabel('$y$ [m]');zlabel('$z$ [m]');
end
parts(:,end)=1/size(parts,1); %all particles have the same initial prob
p.save_robot_state
if (HEUR==2 | HEUR == 3)
    j=10;
else
    j=10;
end
c.Kp=[1000,1000,1000]
noise=0.1
%%
t_tot2=tic
while (j<31)
    c.target=des_angs(1:3)+(rand(1,3)-0.5).*(o.size*1/e.collision_l*15*[0.05,0.05,0.0]);
    if j==30
        c.target=des_angs(1:3)
    end
    g.vis.sim_to_steady
    p_true=Tools.get_true_part(r,e,o,p);
    for i=1:1
        if (j==16 | j==25)
            %      pause
        end
        if(j<15)
            agg=0.1*j/10;%j.^0.4;
            noise=[0.1,02.50,02.50,5000,0.0]*0.5/j;%/j;
        elseif(j<25)
            agg=0.15*j/10;
            noise=[0.1,02.50,02.50,5000,0.0]*0.1/j;%0.5.^((j));
        else
            agg=0.5+j/5;
            noise=[0.1,02.50,02.50,1000,0.0]*0.001;%0.5.^((j));
        end
        
        tic
        
        agg_noise(j,:)=[agg,noise];
        parts=p.motion_model_proper(parts,noise); %bel'=p(x|x(t-1)
        parts_tmp=p.add_noise(parts,noise);
        parts(:,1:end-1)=parts_tmp(:,1:end-1);
        [parts_tmp]=p.get_likelihoods(parts,agg); %p(z|x)
        %TODO:
        parts(:,end)=parts(:,end).*parts_tmp(:,end); %from prob. Rob. book (just likelihood)
        parts=p.resample(parts,0.95);
        p.save_robot_state
        
        toc
        
        if (HEUR==1 | HEUR == 3)
            %restart
            [parts_tmp]=p.get_likelihoods(parts,1); %p(z|x)
            max_lik_n=max(parts_tmp(:,end))
            if(max_lik_n<0.15)
                disp([mat2str(max(parts_tmp(:,end))) 'likelihood restart'])
                parts=p.resample(parts,1.5);
                parts=p.add_noise(parts,[0.1,10,10,100,0.0]);
                [parts]=p.get_likelihoods(parts,1);
                j=15;
                restarted=restarted+1;
                %        return
            end
            if(j==30)
                if(norm(std(parts(:,1:end-1))./mean(parts(:,1:end-1)))>2.0)
                    disp([mat2str(norm(std(parts(:,1:end-1))./mean(parts(:,1:end-1)))) 'std restart'])
                    parts=p.add_noise(parts,[0.05,1,1,100,0.0]);
                    [parts]=p.get_likelihoods(parts,1);
                    j=15;
                    restarted=restarted+10
                    %        return
                end
            end
            
        end
        
        p_true=Tools.get_true_part(r,e,o,p);
        
        if DRAW==true
            
            plot_particles([fg,fg2],p,parts,p_true);
            axes(g.vis.fig_robot);view(60,20);xlabel('$x$ [m]');ylabel('$y$ [m]');zlabel('$z$ [m]');
        end
        if SAVEFIG==true
            %print(fr,[filename2 ,'a' , mat2str(j)],'-dpng') %using this
            frame = getframe(fg);
            im = frame2im(frame);
            frame2 = getframe(fg2);
            im=[im,frame2im(frame2)];
            [imind,cm] = rgb2ind(im,256);
            imwrite(imind,cm,filename,'gif','WriteMode','append');
            
        end
        %axes(g.vis.fig_robot)
        %[az,el]=view;
        %view(az+1,el)
        %    Tools.write_2_pics_to_file([filename2  '.' mat2str(i) mat2str(j) '.png'],fr,g.fig2);
        %  print([filename2 ,'a' , mat2str(j)],'-dpng')
    end
    j=j+1
    if(norm(std(parts(:,1:end-1))./mean(parts(:,1:end-1)))<0.05)
        disp('finished')
        %  break
    end
end
%%
l=j+0
while(j<l)
    agg=10.25;
    noise=0.1;
    %  parts=p.motion_model_proper(parts);
    parts=p.add_noise(parts,noise);
    [parts]=p.get_likelihoods(parts,agg);
    parts=p.resample(parts,1);
    plot_particles([fg,fg2],p,parts,p_true);
    axes(g.vis.fig_robot);view(60,20);xlabel('$x$ [m]');ylabel('$y$ [m]');zlabel('$z$ [m]');
    %  Tools.write_2_pics_to_file([filename2  '.' mat2str(i) mat2str(j) '.png'],fr,g.fig2);
    %print([filename2 ,'a' , mat2str(j)],'-dpng') %using this
    j=j+1
end


%%
time_taken=toc(t_tot)
%[parts]=p.get_likelihoods(parts,1);
[a,b]=max(parts(:,end));
goods=parts(find(parts(:,end)>0.9*a),:);
[a,b]=max(goods(:,end));
%plot_particles([fg,fg2],p,[mean(goods)],p_true, true);
if DRAW==true
    plot_particles([fg,fg2],p,[goods(b,:)],p_true, true);
    axes(g.vis.fig_robot);view(60,20);xlabel('$x$ [m]');ylabel('$y$ [m]');zlabel('$z$ [m]');
    
    %Tools.draw_force(g.vis.fig_robot,r,[mean(goods)])
    Tools.draw_force(g.vis.fig_robot,r,[goods(b,:)])
    
    axes(g.vis.fig_robot_big)
    cla
    copyobj(allchild(g.vis.fig_robot),g.vis.fig_robot_big)
end
if SAVEFIG==true
    %            print(fr,[filename2 ,'a' , mat2str(j)],'-dpng') %using this
    %            print(fr,[filename2 ,'a' , mat2str(j)],'-dpdf') %using this
    %            print(fr,[filename2 ,'a' , mat2str(j)],'-deps') %using this
    frame = getframe(fg);
    im = frame2im(frame);
    frame2 = getframe(fg2);
    im=[im,frame2im(frame2)];
    [imind,cm] = rgb2ind(im,256);
    imwrite(imind,cm,filename,'gif','WriteMode','append');
    
end

[parts_cur]=p.get_likelihoods(parts,1);
[aml,bml]=max(parts_cur(:,end));

res=num2str([p_true;
    goods(b,:);
    mean(goods);
    parts_cur(bml,:);
    (goods(b,:)-p_true)./p_true])
cl=clock;
if SAVEDATA
    fname=['res_h' mat2str(HEUR) '_' mat2str(cl(1)) '_' mat2str(cl(2)) '_' mat2str(cl(3)) '_' o.type '_' mat2str(meas_noise) '_' mat2str(n_par)]
    fid=fopen(fname,'a+');
    fprintf(fid,'%s T=%d (r=%d)\n%s\n%s\n%s\n%s\n%s\n\n',datestr(now),time_taken,restarted,res(1,:),res(2,:),res(3,:),res(4,:),res(5,:))
end
