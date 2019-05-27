%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         Programmatically Datatip Generation          %
%              with MATLAB Implementation              %
%                                                      %
% Author: M.Sc. Eng. Hristo Zhivomirov        08/16/15 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function hDatatip = datatip(hPlot, index, str, varargin)
 
% function: hDatatip = datatip(hPlot, index, str, varargin)
% hPlot - handle of the plot at which the datatip must appear
% index - index into the data matrix (i.e. axis data) at which the datatip must appear
% str - string cell with strings for the datatip's axis labels
% varargin - type 'hold' in the place of varargin if one want to hold the previous datatips
% hDatatip - datatip handle
% update the figure window 
drawnow update
% set data-cursor mode properties
cursorMode = datacursormode(gcf);
set(cursorMode, 'enable', 'on', 'UpdateFcn', {@datatiptxt, str})
% delete the previous datatip(s) if needed
if strcmp(varargin, 'hold')
else
    cursorMode.removeAllDataCursors
end
% create a datatip
hDatatip = cursorMode.createDatatip(hPlot);
 
% set the datatip marker appearance
set(hDatatip, 'Marker','o', 'MarkerSize',10, 'MarkerFaceColor','none',...
              'MarkerEdgeColor','r', 'OrientationMode','auto')
          
% get the axis data
X = get(hPlot, 'XData');
Y = get(hPlot, 'YData');
Z = get(hPlot, 'ZData');
% determine the datatip position
if isvector(X) && isempty(Z)                % for 2D lines
    pos = [X(index) Y(index) 0];
elseif isvector(X)                          % for 3D lines
    pos = [X(index) Y(index) Z(index)];
else                                        % for 3D surfaces
    pos = [X(index(1), index(2)),...
           Y(index(1), index(2)),...
           Z(index(1), index(2))];
end
              
% move the datatip to the position
% uncomment the next line for Matlab R2014a and earlier 
% set(get(hDatatip, 'DataCursor'), 'DataIndex', index, 'TargetPoint', pos)    
set(hDatatip, 'Position', pos)         
updateDataCursors(cursorMode)
% turn Cursor Mode off
set(cursorMode, 'enable', 'off')
end
function text_to_display = datatiptxt(~, hDatatip, str)
% determine current datatip position
pos = get(hDatatip, 'Position');
% form the X and Y axis datatip labels
text_to_display = {[char(str(1)) num2str(pos(1))],...
                   [char(str(2)) num2str(pos(2))]};
% form the Z coordinate datatip label (if exist)
if length(str) > 2
    text_to_display{end+1} = [char(str(3)) num2str(pos(3))];
end
end