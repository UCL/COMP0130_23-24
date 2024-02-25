classdef FigureState < handle
    
    % This class stores a figure together with an associated set of handles
    % for anything within the figure
    
    % TODO: THINK ABOUT ADDING A STATE TO THIS TO ALLOW MAKEQTMOVIE TO RUN
    % AUTOMATICALLY, PERHAPS DURING A "DRAWNOW" PHASE?
    
    properties(Access=public)
        handlesMap;
        figureHandle;
        number;
        name;
        
        axisHandle;
        
        postDrawActions;
    end
    
    methods(Access=public, Sealed=true)

        function this = FigureState(name, number)
            this.name = name;
            this.number = number;
            this.initialize();
        end
 
        function this = initialize(this)
            this.figureHandle = figure(this.number);
            set(this.figureHandle, 'Name', this.name);
            this.reset();
        end

        function this = reset(this)
            this.clf();
            this.postDrawActions = [];
        end

        function this = clf(this)
            % If handle is not valid, then the figure had been closed
            % but the figure manager state not cleared. In that case,
            % intialize it again.
            if (~ishandle(this.figureHandle))
                this.figureHandle = figure(this.number);
                set(this.figureHandle, 'Name', this.name);
            end
            clf(this.figureHandle);
            this.axisHandle = gca;
            this.hold(true);
            this.handlesMap = containers.Map();
        end
        
        function handle = getFigure(this, select)
            handle = this.handle;            
            if ((nargin > 1) && (select == true))
                figure(handle);
            end
        end
        
        function holdState = hold(this, holdState)
            if (holdState == true)
                hold(this.axisHandle, 'on');
            else
                hold(this.axisHandle, 'off');
            end
        end
        
        function this = select(this)
            if (~ishandle(this.figureHandle))
                this.initialize();
            end
            figure(this.figureHandle);
        end

        function this = refreshData(this)
            refreshdata(this.figureHandle);
        end
        
        function this = runPostDrawActions(this)
            for k = 1 : length(this.postDrawActions)
                this.postDrawActions(k).invoke();
            end
        end
        
        function this = setTitle(this, titleString)
            assert(isa(titleString, 'char'), 'figurestate:titlenotstring', ...
                'Title must be a string; method called with an object of class %s', class(titleString));
            % For some reason, the axis handle can become invalidated
            if (ishandle(this.axisHandle) == false)
                figure(this.number);
                this.axisHandle = gca;
            end
            h = get(this.axisHandle, 'Title');
            h.String = titleString;
        end
        
        function axisHandle = getAxisHandle(this)
            axisHandle = this.axisHandle;
        end
        
        function handle = getHandle(this, handleName)
            if (isKey(this.handlesMap, handleName))
                handle = this.handlesMap(handleName);
            else
                handle = [];
            end
        end
        
        function success = addHandle(this, handleName, handle)
            if (isKey(this.handlesMap, handleName))
                success = false;
            else
                this.handlesMap(handleName) = handle;
            end
        end
        
        function this = addPostDrawAction(this, postDrawAction)
            assert(isa(postDrawAction, 'camcalib.graphics.GraphicalAction'), ...
                'figurestate:postdrawactionwrongclass', ...
                'The post draw action must inherit from minislam.graphics.GraphicalAction; the class of the submitted object is %s', ...
                class(postDrawAction));
            this.postDrawActions = cat(2, this.postDrawActions, postDrawAction);
        end        
    end    
end