%Copyright 2018 Joao Bimbo
%Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
%1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
%2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
%THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
classdef GUI < handle
    %GUI Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        fig1
        fig2
        vis
        sld
    end
    
    methods
        function obj=GUI(envir,contro)
            obj.fig2 = figure;
            
            set(obj.fig2,'position',[800,800,800,600]);
            for i=1:length(envir.robot.joint_angles)
                obj.sld(i) = uicontrol(obj.fig2,'Style', 'slider',...
                    'Min',-pi,'Max',pi,'Value',0,...
                    'Position', [70 20+(50*i) 100 20],...
                    'String', mat2str(i), ...
                    'Callback', @obj.add, ...
                    'SliderStep',[0.001,0.01]);
                
                txt = uicontrol(obj.fig2,'Style','text',...
                    'Position',[10 20+(50*i) 50 20],...
                    'String', ['joint' mat2str(i)]);
                
            end
            set(obj.fig2,'ButtonDownFcn',@obj.update)
            obj.fig1 = axes;
            set(obj.fig1,'Position',[0.35,0.05,0.6,0.9]);            
            obj.vis=Visualizer(obj.fig1,envir,contro);
            obj.update(0,0);
        end
        
        
        function update(obj,a,b)
            length(obj.sld)
            for i=1:length(obj.sld)
                set(obj.sld(i),'Value',obj.vis.environm.robot.joint_angles(i))
            end
            obj.vis.draw;
            
        end
        
        function s=add(obj,a,b)
            obj.vis.environm.robot.set_angle(str2double(a.String),a.Value);
            obj.vis.draw;
        end
    end
    
end

