%Copyright 2018 Joao Bimbo
%Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
%1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
%2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
%THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
function ticks_in_latex(fig_h,ax,ticknames,offsets)
HorizontalOffset = offsets(1);
verticalOffset = offsets(2);

set(ax,'Box','off')
set(ax, 'Color', 'None')
set(ax,'yticklabel',[], 'xticklabel', []) %Remove tick labels
yTicks = get(ax,'ytick');
xTicks = get(ax, 'xtick');
axis(ax);
ax_h = axis; %Get left most x-position

for j = 1:length(yTicks)
    %Create text box and set appropriate properties
    text(ax_h(1) - HorizontalOffset,yTicks(j),['$' num2str( yTicks(j)) '$'],...
        'HorizontalAlignment','Right','interpreter', 'latex','FontSize',14);
end
minY = min(yTicks);
axis(ax);
if(~isempty(ticknames))
    for xx = 1:length(xTicks)
        %Create text box and set appropriate properties
        %text(xTicks(xx), minY - verticalOffset, ['$' ticknames{xx} '$'],...
        %    'HorizontalAlignment','Right','interpreter', 'latex');        
        tx(xx)=text(xTicks(xx),minY - verticalOffset,['$' ticknames{xx} '$'],...
            'HorizontalAlignment','center','interpreter','latex','FontSize',14);
    end
end
set(fig_h,'Color',[1,1,1])


end