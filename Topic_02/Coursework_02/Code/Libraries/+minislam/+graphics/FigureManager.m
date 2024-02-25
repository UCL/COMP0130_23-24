classdef FigureManager < handle
   
    methods (Access=public, Static)
        function obj = getInstance()            
            persistent localInstance;
            if isempty(localInstance) || ~isvalid(localInstance)
                localInstance = minislam.graphics.FigureManager();
            end
            obj = localInstance;
        end

        % Get the figure handle for a named figure. If the figure does not
        % exist, create
        function figureState = getFigure(figureName, createIfDoesNotExist)
            if (nargin == 1)
                createIfDoesNotExist = true;
            end
            queryFigureState = minislam.graphics.FigureManager.getInstance().getOrAllocateFigure(figureName, createIfDoesNotExist);
            if (nargout == 1)
                figureState = queryFigureState;
            end
        end
        
        function clf()
            trackerlib.graphics.FigureManager.getInstance().clfAllFigures();
        end
        
        function allFigures = getAllFigures()
            allFigures = trackerlib.graphics.FigureManager.getInstance().figureMap.values();
        end
        
        function initializePostDrawActions()
            trackerlib.graphics.FigureManager.getInstance().initializePostDrawActionsOnAllFigures();
        end

        function runPostDrawActions()
            trackerlib.graphics.FigureManager.getInstance().runPostDrawActionsOnAllFigures();
        end
    end

    properties(Access=protected)
        nextFreeFigureNumber = 1;
        figureMap;
    end
    
    methods(Access=public)
        function figureState = getOrAllocateFigure(this, figureName, createIfDoesNotExist)
            if (isKey(this.figureMap, figureName))
                figureState = this.figureMap(figureName);
                figureState.select();
            elseif (createIfDoesNotExist == true)
                figureState = minislam.graphics.FigureState(figureName, this.nextFreeFigureNumber);
                this.figureMap(figureName) = figureState;
                this.nextFreeFigureNumber = this.nextFreeFigureNumber + 1;
            else
                figureState = [];
            end
        end
        
        function clfAllFigures(this)
            valueSet = this.figureMap.values();
            for f = 1 : length(valueSet)
                valueSet{f}.clf();
            end
        end
        
        function initializePostDrawActionsOnAllFigures(this)
            valueSet = this.figureMap.values();
            for f = 1 : length(valueSet)
                valueSet{f}.initializePostDrawActions();
            end
        end

        function runPostDrawActionsOnAllFigures(this)
            valueSet = this.figureMap.values();
            for f = 1 : length(valueSet)
                valueSet{f}.runPostDrawActions();
            end
        end

    end
    
    methods(Access=protected)
        
        function this = FigureManager()
            this.figureMap = containers.Map();
        end
    end
end