% This class stores events in an ordered queue. Sorting is based on
% increasing time value.

classdef OrderedEventQueue < handle

    properties(Access = protected)
        sortedEvents;
    end
        
    methods(Access = public)
        
        function this = OrderedEventQueue()
            this.sortedEvents = {};
        end
        
        function clear(this)
            this.sortedEvents = {};
        end
        
        function insert(this, newEvents)
            
            % Handle the case of a single event first
            if (iscell(newEvents) == false)
                this.insertEvent(newEvents);
                return
            end
            
            % Now handle the cell array
            for c = 1 : length(newEvents)
                this.insertEvent(newEvents{c});
            end
        end
        
        function sortedEvents = events(this)
            sortedEvents = this.sortedEvents;
        end        
    end
    
    methods(Access = protected)
        function insertEvent(this, newEvent)
            
            % For convenience turn into a cell array if not done so already
            
            assert(isa(newEvent, 'minislam.event_types.Event') == true , ...
                'minislam:event_generator:orderedeventqueue:eventobjectofwrongtype', ...
                'The event object is of class %s; it should inherit from minislam.event_types.Event', ...
                class(newEvent));
            
            % If the queue is empty, add and return
            if (isempty(this.sortedEvents) == true)
                this.sortedEvents = {newEvent};
                return
            end
            
            % Manually search through the list
            for e = 1 : length(this.sortedEvents)
                if newEvent.time <= this.sortedEvents{e}.time
                    this.sortedEvents = {this.sortedEvents{1:e-1}, newEvent, this.sortedEvents{e:end}};
                    return
                end
            end
            this.sortedEvents = {this.sortedEvents, newEvent};            
        end
  
    end
end
