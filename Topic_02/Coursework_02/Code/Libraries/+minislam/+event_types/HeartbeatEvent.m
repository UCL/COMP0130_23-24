% This event contains no data. When it is received by a localization
% algorithm, it triggers updating to the current time.

classdef HeartbeatEvent < minislam.event_types.Event

    methods(Access = public)
        
        function this = HeartbeatEvent(time)
            this = this@minislam.event_types.Event(time, minislam.event_types.EventTypes.HEARTBEAT, [], []);
        end
    end
end
