function h = BYPASS_DIST
% BYPASS_DIST controls how long by walking forward
% the robot bypasses current obstacle

% If this value is too much, we are too far away from
% the obstacle, and stand little chance of getting
% back to the obstacle (because we're circumnavigating).

% And if it's too little, the robot won't move at all
% due to frictions.

h = 0.05;
end

