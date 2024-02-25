classdef SimulatorState < handle
   
     % This structure stores simulator state results
     
     properties(Access = public)
        
         % The current time (in s)
         currentTime;
         
         % The true state of the vehicle; use for debugging but NOT in your
         % filter. State is (x,y,psi)
         xTrue;

         % The true map; use for debugging but NOT in your
         % filter. State is (m1x, m1y, m2x, m2y, ....)
         mTrue;         

         % The true control inputs (speed, angle turn rate)
         uTrue;
         
         % The waypoints. Order is [w1 w2 w3 ...] where each waypoint
         % is w1 = (x1, y1)'
         waypoints;
         
     end
end