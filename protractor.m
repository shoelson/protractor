classdef protractor < handle
    % protractor
    %    Adds an interactive angle- and distance-measurement tool to the
    %    current axes. Currently build on Image Processing Toolbox
    %    functionality, and works well on image-containing axes. But it
    %    also works on non-image axes.
    %
    % As you drag from any vertex, the labels change to indicate the current
    % angle between the two lines intersecting at the middle vertex, and the
    % distances between vertex 1 and 2, and between vertex 2 and 3.
    %
    % A rotation handle allows you to rotate the object around the middle
    % vertex.
    %
    % The color of the protractor will change to green when the angle is a
    % multiple of 90; it will be striped green-red when it is a multiple of 45;
    % and it will be red otherwise.
    %
    % Click on the distance labels to change the scale.
    %
    %
    % SYNTAX:
    % h = protractor;
    %    Adds protractor centered on current axes. Returns in h the handle to a
    %    protractor object.
    %
    % h = protractor(PV_Pairs);
    %    Allows optional specification of parameter-value pairs, as specified
    %    below:
    %
    % h = protractor('parentAx', ax)
    %    Allows specification of parent axes.
    %
    % h = protractor('position', position);
    %    Allows specification of initial position, specified as a 3x2 array in
    %    the form [x1 y1; x2 y2; x3 y3].
    %
    % METHODS:
    % (Methods are also available via right-click uicontextmenus.)
    %
    % rotateProtractor
    %    h.rotateProtractor(thetaDegrees) (rotateProtractor(h, thetaDegrees)
    %    allows programmatic rotation of the protractor by thetaDegrees.
    %
    % snap45
    %    h.snap45 (or snap45(h)) allows programmatically setting the angle to
    %    45 degrees.
    %
    % snap90
    %    h.snap90 (or snap90(h)) allows programmatically setting the angle to
    %    90 degrees.
    %
    % snapArbitrary
    %    h.snapArbitrary (or snapArbitrary(h)) will prompt the user to enter a
    %    theta value (in degrees), and will snap the protractor to that angle.
    %
    % toggleDistanceLabels
    %    Facilitates toggling the visibility of the distance labels on and off.
    %
    %
    % % EXAMPLES
    %
    % % Example1: On an Image
    % % %(Requires the Image Processing Toolbox)
    % img = imread('gantrycrane.png');
    % imshow(img)
    % h = protractor
    %
    %
    % % Example 2: Programmatically placed on an image:
    % % %(Requires the Image Processing Toolbox)
    % rng(5); % For repeatability
    % bw = false(100);
    % inds = randi([0 numel(bw)], 3, 1);
    % bw(inds) = true;
    % bw = imdilate(bw, strel('disk', 1));
    % [ys, xs] = ind2sub(size(bw), inds);
    % imshow(bw);
    % % Randomly select 3 points, measure the distance between them
    % h = protractor('ParentAx', imgca, 'position', [xs ys]);
    %
    %
    % % % Example 3: In a plot
    % % %(Still requires the Image Processing Toolbox)
    % x = randi([-5, 5], 10, 1);
    % y = randi([-4, 4], 10, 1);
    % plot(x, y, 'r.', 'Marker', '.', 'Markersize', 32, 'Color', 'c')
    %   % NOTE: 'axis equal' is for visualization--makes the actual
    %   %  angle match the apparent angle. But it's not strictly necessary!
    % axis equal
    % h = protractor('position', [x(1:3), y(1:3)]);
    % xlim([-8, 8])
    % ylim([-8, 8]);
    %
    % Brett Shoelson, PhD
    % bshoelso@mathworks.com
    % 09/24/2020
    %
    % See Also: drawpolyline 
    
    % Copyright 2020 The MathWorks, Inc.
    
    properties
        d1
        d2
        distanceLabel1
        distanceLabel2
        isReversed
        mainHandle
        parentAx
        position
        rotationHandle
        thetaDegrees
    end
    
    methods % INSTANTIATE:
        
        function obj = protractor(varargin)
            % Implement input arguments
            [obj.isReversed, obj.parentAx, ...
                obj.position] = ...
                parseInputs(varargin{:});
            
            function [isReversed, parentAx, position] = ...
                    parseInputs(varargin)
                % Setup parser with defaults
                parser = inputParser;
                parser.CaseSensitive = false;
                parser.addParameter('parentAx', []);
                parser.addParameter('position', []);
                % Parse input
                parser.parse(varargin{:});
                % Assign outputs
                r = parser.Results;
                [parentAx, position] = ...
                    deal(r.parentAx, r.position);
                if isempty(parentAx)
                    parentAx = gca;
                end
                isReversed = parentAx.YDir == "reverse";
                if isempty(position)
                    xRange = diff(parentAx.XLim);
                    yRange = diff(parentAx.YLim);
                    % Get center point of image for initial positioning
                    y2 = mean(parentAx.YLim);
                    x2 = mean(parentAx.XLim);
                    % Position first point vertically above the middle
                    x1 = x2;
                    if isReversed
                        y1 = y2 - yRange/4;
                    else
                        y1 = y2 + yRange/4;
                    end
                    x3 = x2 + xRange/4;
                    y3 = y2;
                    position = [x1, y1; x2, y2; x3, y3];
                else
                    if ~all(size(position)==[3 2])
                        error('protractor: Invalid specification of position; must be [3 x 2]');
                    end
                end
            end %parseInputs
            %
            
            % rotationHandle
            % Need to create rotationHandle first because uistack isn't working (G2337827)
            [x1, y1, x2, y2, x3, y3] = parsePosition(obj);
            obj.rotationHandle = drawline(obj.parentAx, ...
                'Position', [x2 y2; x3 y3], ...
                'StripeColor', 'w', ...
                'LineWidth', 0.5, ...
                'HandleVisibility', 'Callback', ...
                'Tag', 'RotationHandle', ...
                'Deletable', false, ...
                'InteractionsAllowed', 'reshape');
            % Userdata for text labels
            if obj.isReversed
                textUD.Units = 'pixels';
            else
                textUD.Units = 'units';
            end
            textUD.ScalePerPixel = 1;
            obj.rotationHandle.UserData = textUD;
            
            % obj
            obj.mainHandle = drawpolyline('Parent', obj.parentAx,...
                'Position', obj.position, ...
                'Label', '', ...
                'Tag', 'obj', ...
                'Color', [0 0.8 0] );
            
            % CONTEXT MENU
            % Add empty context menu to replace default menu
            cmenu = uicontextmenu;
            uimenu(cmenu, ...
                'Label', 'Delete', ...
                'Callback', ...
                @(src, evt) deleteTool(obj));
            uimenu(cmenu, ...
                'Label', 'Snap horizontal', ...
                'Callback', @(src, evt) snapHorizontal(obj));
            uimenu(cmenu, ...
                'Label', 'Snap vertical', ...
                'Callback', @(src, evt) snapVertical(obj));
            uimenu(cmenu, ...
                'Label', 'Snap to 45 degrees', ...
                'Callback', @(src, evt) snap45(obj));
            uimenu(cmenu, ...
                'Label', 'Snap to 90 degrees', ...
                'Callback', @(src, evt) snap90(obj));
            uimenu(cmenu, ...
                'Label', 'Snap to ? degrees', ...
                'Callback', @(src, evt) snapArbitrary(obj));
            uimenu(cmenu, ...
                'Label', 'Toggle Distance Labels', ...
                'Callback', @(src, evt) toggleDistanceLabels(obj));
            uimenu(cmenu, ...
                'Label',  'Export to Workspace...', ...
                'Callback', @(src, evt) exportToWorkspace(obj));
            obj.mainHandle.UIContextMenu = cmenu;
            
            % LISTENERS
            % Intercept attempts to add or remove vertices
            addlistener(obj.mainHandle, ...
                'AddingVertex', ...
                @(src, evt) storePositionInUserData(obj));
            addlistener(obj.mainHandle, ...
                'VertexAdded', ...
                @(src, evt) recallPositionInUserData(obj));
            addlistener(obj.mainHandle, ...
                'DeletingVertex', ...
                @(src, evt) storePositionInUserData(obj));
            addlistener(obj.mainHandle, ...
                'VertexDeleted', ...
                @(src, evt) recallPositionInUserData(obj));
            addlistener(obj.mainHandle, ...
                'MovingROI', ...
                @(src, evt) update(obj));
            addlistener(obj.rotationHandle,'MovingROI', ...
                @(src, evt) rotatingTool(obj));
            
            obj.distanceLabel1 = text(0, 0, '', ...
                'BackgroundColor', 'w', ...
                'FontSize', 8, ...
                'tag', 'Label1', ...
                'HitTest','on',...
                'Clipping','on', ...
                'ButtondownFcn', @(varargin)textClick(obj));
            obj.distanceLabel2 = text(0, 0, '', ...
                'BackgroundColor', 'w', ...
                'FontSize', 8, ...
                'tag', 'Label2', ...
                'Clipping','on', ...
                'ButtondownFcn', @(varargin)textClick(obj));
            
            % UPDATE
            update(obj);
        end %protractor (Instantiation)
        
    end
    
    methods (Access = public)
        
        function deleteTool(obj)
            delete(obj.rotationHandle)
            delete(findall(obj.parentAx, 'Tag', 'Label1'))
            delete(findall(obj.parentAx, 'Tag', 'Label2'))
            delete(obj.mainHandle)
        end
        
        function exportToWorkspace(h, varargin)
            varnames = {'thetaDeg', 'd1', 'd2', 'vertexPositions'};
            labels = varnames;
            items = {h.thetaDegrees, h.d1, h.d2, h.position()};
            export2wsdlg(labels, varnames, items, 'Export to Workspace');
        end %exportToWorkspace

        function posNew = rotateProtractor(obj, thetaDeg, commandLine)
            if nargin < 3
                commandLine = true;
            end
            pos = obj.position;
            %Rotate about the origin, then shift:
            dOrigin = [0 0] - pos(2, :);
            posShifted = pos + dOrigin;
            thetaRad = thetaDeg * pi/180;
            [v1ThetaRad, v1Rho] = cart2pol(posShifted(1, 1), posShifted(1, 2));
            [v2ThetaRad, v2Rho] = cart2pol(posShifted(3, 1), posShifted(3, 2));
            [v1xShifted, v1yShifted] = pol2cart(v1ThetaRad + thetaRad, v1Rho);
            [v2xShifted, v2yShifted] = pol2cart(v2ThetaRad + thetaRad, v2Rho);
            posNew = [v1xShifted, v1yShifted;
                0 0;
                v2xShifted, v2yShifted] - dOrigin;
            if commandLine
                obj.mainHandle.Position = posNew;
                update(obj)
            end
        end % rotateProtractor
        
        function snap45(obj)
            targetTheta = 45;
            %             if ~obj.isReversed
            %                 targetTheta = -targetTheta;
            %             end
            dTheta = round(obj.thetaDegrees - targetTheta, 8);
            if dTheta == 0
                return
            end
            posNew = rotateProtractor(obj, dTheta, false);
            posNew = [posNew(1, :); obj.position(2:3, :)];
            obj.mainHandle.Position = posNew;
            % Trigger angle update
            update(obj);
            if round(obj.thetaDegrees - 45, 8) ~= 0
                % This is easier than figuring out which direction is
                % 'relatively positive'
                dTheta = -2 * dTheta;
                posNew = rotateProtractor(obj, dTheta, false);
                posNew = [posNew(1, :); obj.position(2:3, :)];
                obj.mainHandle.Position = posNew;
                % Trigger angle update
                update(obj);
            end
        end %snap45
        
        function snap90(obj)
            targetTheta = 90;
            dTheta = round(obj.thetaDegrees - targetTheta, 8);
            if dTheta == 0
                return
            end
            posNew = rotateProtractor(obj, dTheta, false);
            posNew = [posNew(1, :); obj.position(2:3, :)];
            obj.mainHandle.Position = posNew;
            % Trigger angle update
            update(obj);
            if round(obj.thetaDegrees - 90, 8) ~= 0
                % This is easier than figuring out which direction is
                % 'relatively positive'
                dTheta = -2 * dTheta;
                posNew = rotateProtractor(obj, dTheta, false);
                posNew = [posNew(1, :); obj.position(2:3, :)];
                obj.mainHandle.Position = posNew;
                % Trigger angle update
                update(obj);
            end
            
        end %snap45
        
        function snapArbitrary(obj, targetTheta)
            if nargin < 2
                targetTheta = inputdlg({'Enter theta (degrees):'},...
                    'Theta (Degrees)', ...
                    [1 40], ...
                    {'0'});
                targetTheta = str2double(targetTheta{1});
            end
            dTheta = round(obj.thetaDegrees - targetTheta, 8);
            if dTheta == 0
                return
            end
            posNew = rotateProtractor(obj, dTheta, false);
            posNew = [posNew(1, :); obj.position(2:3, :)];
            obj.mainHandle.Position = posNew;
            % Trigger angle update
            update(obj);
            
            if round(obj.thetaDegrees - targetTheta, 8) ~= 0
                % This is easier than figuring out which direction is
                % 'relatively positive'
                dTheta = -2 * dTheta;
                posNew = rotateProtractor(obj, dTheta, false);
                posNew = [posNew(1, :); obj.position(2:3, :)];
                obj.mainHandle.Position = posNew;
                % Trigger angle update
                update(obj);
            end
        end %snap45
        
        function toggleDistanceLabels(obj)
            
            isvis = get(obj.distanceLabel1, 'Visible');
            if strcmp(isvis, 'on')
                obj.distanceLabel1.Visible = 'off';
                obj.distanceLabel2.Visible = 'off';
            else
                obj.distanceLabel1.Visible = 'on';
                obj.distanceLabel2.Visible = 'on';
            end
        end %toggleDistanceLabels
        
    end
    
    methods (Access = private)
        
        function [x1, y1, x2, y2, x3, y3] = parsePosition(obj)
            pos = obj.position;
            x1 = pos(1, 1);
            x2 = pos(2, 1);
            x3 = pos(3, 1);
            y1 = pos(1, 2);
            y2 = pos(2, 2);
            y3 = pos(3, 2);
        end %parsePosition
        
        function recallPositionInUserData(obj)
            % Restore the previous Position to prevent users from adding/removing
            % vertices
            obj.mainHandle.Position = obj.mainHandle.UserData;
        end %recallPositionInUserData
        
        function rotatingTool(obj)
            % Let's calculate its angle, and rotate obj.mainHandle (the polyline) so that its
            % second line matches the slope.
            rHPos = obj.rotationHandle.Position;
            dY = rHPos(2, 2) - rHPos(1, 2);
            dX = rHPos(2, 1) - rHPos(1, 1);
            rHThetaD = atand(dY/dX);
            hPos = obj.mainHandle.Position;
            
            hThetaD = atand((hPos(3, 2) - hPos(2, 2))/(hPos(3, 1) - hPos(2, 1)));
            dThetaD = hThetaD - rHThetaD;
            
            % If the x-coord of the rotationHandle endpoint is on opposite sides of the
            % central point from the x-coord of line2, we're 180 degrees out of sync!
            if sign(rHPos(2, 1) - rHPos(1, 1)) ~= sign(hPos(3, 1) - hPos(2, 1))
                dThetaD = 180 + dThetaD;
            end
            
            dThetaD = dThetaD + eps; %I do this so we don't see "-0".
            if dThetaD < 0
                dThetaD = dThetaD + 360;
            end
            vertices = rotateObject(obj.mainHandle, dThetaD, ...
                'centerOfRotation', obj.mainHandle.Position(2, :));
            vertices = vertices(1:3, :);
            obj.mainHandle.Position = vertices;
            update(obj);
        end %rotatingTool
        
        function update(obj)
            obj.position = obj.mainHandle.Position;
            p = obj.position;
            [RotHndlX2, RotHndlY2] = calculateRotationHndlPosition(p);
            obj.rotationHandle.Position = [p(2, 1) p(2, 2); RotHndlX2, RotHndlY2];
            % Find the angle
            v1 = [p(1, 1) - p(2, 1),  p(1, 2) - p(2, 2)];
            v2 = [p(3, 1) - p(2, 1),  p(3, 2) - p(2, 2)];
            theta = acos(dot(v1, v2)/(norm(v1)*norm(v2)));
            if isnan(theta)
                theta = 0;
            end
            % Convert it to degrees
            obj.thetaDegrees = (theta * (180/pi));
            % Update the Label to display the angle
            obj.mainHandle.Label = sprintf('%1.1f degrees', obj.thetaDegrees);
            approxAngle = round(obj.thetaDegrees, 1);
            if ismember(approxAngle, [0, 90, 180, 270])
                obj.mainHandle.Color = [0 0.8 0];
                obj.mainHandle.StripeColor = 'none';
            elseif ismember(approxAngle, [45, 135])
                obj.mainHandle.Color = [0 0.8 0];
                obj.mainHandle.StripeColor = [0.8 0.2 0.2];
            else
                obj.mainHandle.Color = [0.8 0.2 0.2];
                obj.mainHandle.StripeColor = 'none';
            end
            obj.d1 = norm([diff(p(1:2, 1)), diff(p(1:2, 2))]);
            obj.d2 = norm([diff(p(2:3, 1)), diff(p(2:3, 2))]);
            
            % Update distance strings:
            p1x = mean(p(1:2, 1));
            p1y = mean(p(1:2, 2));
            p2x = mean(p(2:3, 1));
            p2y = mean(p(2:3, 2));
            ud = obj.rotationHandle.UserData;
            str1 = sprintf('  %0.1f %s', obj.d1*ud.ScalePerPixel, ud.Units);
            str2 = sprintf('  %0.1f %s', obj.d2*ud.ScalePerPixel, ud.Units);
            set(obj.distanceLabel1, ...
                'Position', [p1x p1y], ...
                'String', str1);
            set(obj.distanceLabel2, ...
                'Position', [p2x p2y], ...
                'String', str2);
            function [RotHndlX2, RotHndlY2] = calculateRotationHndlPosition(pos)
                offset = 0.2;
                dy = offset * (pos(3, 2) - pos(2, 2));
                dx = offset * (pos(3, 1) - pos(2, 1));
                RotHndlX2 = pos(3, 1) + dx;
                RotHndlY2 = pos(3, 2) + dy;
            end %calculateRotationHndlPosition
            
        end %update
        
        function snapHorizontal(obj)
            % Which line segment was selected?
            cp = get(gca, 'currentPoint');
            cp = cp(1, 1:2);
            xl = obj.parentAx.XLim;
            yl = obj.parentAx.YLim;
            if cp(1) < xl(1) || cp(1) > xl(2) || cp(2) < yl(1) || cp(2) > yl(2)
                % Programmatic call!
                ind = 1;
            else
                hpos = obj.mainHandle.Position;
                centroids = [mean(hpos(1:2, :)); mean(hpos(2:3, :))];
                d = @(x, y) sqrt((x(1) - y(1))^2 + (x(2) - y(2))^2);
                ds = [d(centroids(1, :), cp); d(centroids(2, :), cp)];
                [~, ind] = min(ds);
            end
            switch ind
                case 1
                    obj.mainHandle.Position (1, 2) = obj.mainHandle.Position(2, 2);
                case 2
                    obj.mainHandle.Position(3, 2) = obj.mainHandle.Position (2, 2);
            end
            % Trigger angle update
            obj.position = obj.mainHandle.Position;
            update(obj);
        end %snapHorizontal
        
        function snapVertical(obj)
            % Which line segment was selected?
            cp = get(gca, 'currentPoint');
            cp = cp(1, 1:2);
            hpos = obj.mainHandle.Position;
            centroids = [mean(hpos(1:2, :)); mean(hpos(2:3, :))];
            d = @(x, y) sqrt((x(1) - y(1))^2 + (x(2) - y(2))^2);
            ds = [d(centroids(1, :), cp); d(centroids(2, :), cp)];
            [~, ind] = min(ds);
            switch ind
                case 1
                    obj.mainHandle.Position (1, 1) = obj.mainHandle.Position(2, 1);
                case 2
                    obj.mainHandle.Position(3, 1) = obj.mainHandle.Position (2, 1);
            end
            % Trigger angle update
            obj.position = obj.mainHandle.Position;
            update(obj);
        end %snapVertical
        
        function storePositionInUserData(obj)
            % Prevent Users From Adding or Removing Vertices The polyline allows users
            % to add and remove vertices; however, an angle measurement tool assumes
            % that there are only three vertices at any time. We want to add listeners
            % to intercept when a user is attempting to add or remove a vertex. In the
            % callback for the 'AddingVertex' and DeletingVertex' events, the vertex of
            % interest has not yet been added or deleted. We will cache the current
            % polyline position in the UserData property and restore it in the callback
            % for the 'VertexAdded' and 'VertexDeleted' events. This will prevent the
            % user from interactively adding or removing vertices for the polyline.
            %
            % Before a vertex is added/removed, store the Position in the UserData
            % property
            obj.mainHandle.UserData = obj.position;
        end %storePositionInUserData
        
        function textClick(obj)
            obj.rotationHandle.UserData;
            currentObj = gcbo;
            objTag = get(currentObj, 'tag');
            currentString = strtrim(currentObj.String);
            spacePos = find(currentString == 32);
            d = currentString(1:spacePos - 1);
            u = currentString(spacePos + 1:end);
            prompt = {sprintf('Enter a new value for string %s', objTag),...
                'Enter new units:'};
            newVals = inputdlg(prompt,'Change Scale/Units',[1 50], {d, u});
            if isempty(newVals)
                return
            end
            p = obj.position;
            switch objTag
                case 'Label1'
                    dPixels = norm([diff(p(1:2, 1)), diff(p(1:2, 2))]);
                case 'Label2'
                    dPixels = norm([diff(p(2:3, 1)), diff(p(2:3, 2))]);
            end
            ud.Units = newVals{2};
            ud.ScalePerPixel = str2double(newVals{1})/dPixels;
            obj.rotationHandle.UserData = ud;
            update(obj)
        end %textClick
        
    end
    
    methods (Static)
        % This is required by handle class
        function pos = getPosition()
            pos = [];
            disp('Use hndl.position')
        end
    end
    
end

