%Copyright 2018 Joao Bimbo
%Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
%1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
%2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
%THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
function plot_particles(figs,p,parts,p_true,final)

if(nargin==4)
    pl_final=false;
else
    pl_final=final;
end
set(0,'defaulttextinterpreter','latex')

%colors for likelihood:
parts_cst=p.get_likelihoods(parts,2);


axes(figs(1));
[az,el]=view;
views=[0,0;0,0;0,0;0,0];
sz_p=size(parts);
col_cost=sz_p(2);

parts(:,col_cost+1)=sqrt(parts(:,3).^2+parts(:,4).^2);
p_true(:,col_cost+1)=sqrt(p_true(:,3).^2+p_true(:,4).^2);

colors=zeros(sz_p(1),3);
colors(:,1:2)=[parts(:,col_cost)*sz_p(1),1-parts(:,col_cost)*sz_p(1)];
%sz=5*ones(sz_p(1),1);
sz=2*parts_cst(:,col_cost);



view_limits=[0,0;
    0,0.75;
    -150,150;
    -150,150;
    0,5000;
    %0,0.25;
    0,1;
    0,200;];

plotted=[3,4,5,col_cost;
         2,5,2,col_cost
         2,3,5,col_cost];

 plotted=[2,col_cost+1,5,col_cost;
          2,5,6,col_cost
          5,6,col_cost+1,col_cost];




labels={'$n$','$l [m]$','$Fx$','$Fy$','$K [N / m]$','$cost$','$F [N]$'};
%labels={'n','l [m]','Fx','Fy','$K [N / m]$','r [m]','cost','F [N]'};

if(length(figs)>3)
   sz= length(figs)-1;
else
    sz= length(figs);
end

for i=1:sz
    axes(figs(i));
    cla
    hold on
    scatter(parts(:,plotted(i,1)),...
    parts(:,plotted(i,2)),...
    ...parts(:,plotted(i,3)),...
    sz,colors)
    scatter(p_true(:,plotted(i,1)),...
        p_true(:,plotted(i,2)),...
    ...view_limits(plotted(i,3),2),...
        40,[0.95,0.95,0],'Marker','o','MarkerFaceAlpha',0.6,'MarkerFaceColor',[0.9,0.9,0]);
    
    %plot3(p_true(:,plotted(i,1)),p_true(:,plotted(i,2)),view_limits(plotted(i,3),:),'Marker','o','MarkerFaceColor',[0.95,0.95,0],'Color',[0,0,0],'MarkerSize',6);
    %plot3(p_true(:,plotted(i,1)),p_true(:,plotted(i,2)),p_true(:,plotted(i,3)),'Marker','o','MarkerFaceColor',[0.95,0.95,0],'Color',[0,0,0],'MarkerSize',6);
    axis([view_limits(plotted(i,1),:),view_limits(plotted(i,2),:),view_limits(plotted(i,3),:)]);
    if(pl_final)
        [~,b]=max(parts(:,col_cost));
        [~,a]=max(parts(:,plotted(i,3)));
        plot(parts(b,plotted(i,1)),...
            parts(b,plotted(i,2)),...
        ...view_limits(plotted(i,3),2),
            'bx','LineWidth',4,'MarkerSize',5);
        plot(p_true(:,plotted(i,1)),...
            p_true(:,plotted(i,2)),...
            ...view_limits(plotted(i,3),2),
            'k.','MarkerSize',20);
    end
    xlabel(labels{plotted(i,1)},'Interpreter','LaTeX','FontSize', 14);
    ylabel(labels{plotted(i,2)},'Interpreter','LaTeX','FontSize', 14);
    %zlabel(labels{plotted(i,3)},'Interpreter','LaTeX','FontSize', 14);
    %view(az+views(i,1),el+views(i,2))
    drawnow
    hold off
end

if(length(figs)>3)
    axes(figs(end));
    hist(parts(:,col_cost));
    %hist(parts(:,col_cost),[0:0.05:1.1]);
    %axis([0,1.1,0,sz_p(1)])
    xlabel('$L(p)$','Interpreter','LaTeX','FontSize', 14);
    ylabel('$n$','Interpreter','LaTeX','FontSize', 14);
end

end