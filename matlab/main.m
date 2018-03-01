close all
clear
DRAW=true;
SAVEFIG=false;
SAVEDATA=false;

if SAVEFIG && ~isdir('fig_trial')
    mkdir fig_trial
end

HEUR=3; %Choose which Heuristics
rng shuffle
restarted=0;
r=Robot([0.2,0.74,0.3],[ -0.24   -0.85   -0.1000]); %Create a robot
des_angs=r.joint_angles;
[~,~,T]=r.fwd;
o=Obstacle([0.32,0.2,0.4],2500,0.08); %sphere random
e=Environment(r,o);
c=Controller(r,[1500,1500,1500],[0,0,0],[0,0,0]);
c.cur_controller='impedance';
g=GUI(e,c);
if DRAW
    g.vis.draw
end
meas_noise=0.0;
n_par=2500;
p=Perception(r,140,5000,0.3,e,o,meas_noise);

%%
c.target=des_angs;
g.vis.sim_to_steady

%%
p.save_robot_state
if (HEUR==0 || HEUR == 1)
    parts=p.init_parts(n_par);
else
    parts=p.init_parts_heur(n_par,2);
end
p_true=Tools.get_true_part(r,e,o,p);
if DRAW==true
    fr=figure;
    fg=subplot(2,2,1);
    fg2=subplot(2,2,3);
    fg3=subplot(1,2,2);
    g.vis.fig_robot=fg3;
    
    plot_particles([fg,fg2],p,parts,p_true);
    axes(g.vis.fig_robot);view(60,20);xlabel('$x$ [m]');ylabel('$y$ [m]');zlabel('$z$ [m]');
    g.vis.draw
    filename = 'testnew51.gif';
    filename2 = 'fig_trial/figure';
    set(fr,'Position',[200,590,600,400])
    
    if SAVEFIG==true
        frame = getframe(fg);
        im = frame2im(frame);
        frame2 = getframe(fg2);
        im=[im,frame2im(frame2)];
        [imind,cm] = rgb2ind(im,256);
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
    end
end

[parts]=p.get_likelihoods(parts,1);
p.save_robot_state
k=1;
c.target=des_angs(1:3);


t_tot=tic;
if (HEUR==2 || HEUR == 3)
    j=9;
else
    j=1;
end
i=1;
%%
while(j<10)
    agg=0.05*j;
    noise=[01.5,5,5,5000,0.025]/j;
       parts=p.add_noise(parts,noise);
    [parts]=p.get_likelihoods(parts,agg);
    parts=p.resample(parts,1);
    if DRAW==true
        plot_particles([fg,fg2],p,parts,p_true);
        axes(g.vis.fig_robot);view(60,20);xlabel('$x$ [m]');ylabel('$y$ [m]');zlabel('$z$ [m]');
        
        if SAVEFIG==true
            frame = getframe(fg);
            im = frame2im(frame);
            frame2 = getframe(fg2);
            im=[im,frame2im(frame2)];
            [imind,cm] = rgb2ind(im,256);
             imwrite(imind,cm,filename,'gif','WriteMode','append'); 
        end
    end
    j=j+1
end

parts_saved=parts;
%%
%save('current')
main2
