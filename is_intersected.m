function b = is_intersected(old_pose)
% judege whether or not Roomba is on m-line
% return 1 if Roomba is on m-line
% return 0 otherwise

global goal_coord % [4 0] destination

current_pos = pos_from_ht(old_pose); % current position

% vector_1: vector points from startpoint of Roomba to current position
% vector_2: vector points from startpoint of Roomba to destination
%           also called as "m-line"
% calculate angle between vector_1 and vector_2
angle = acos(dot(goal_coord,current_pos)/...
        (norm(goal_coord)*norm(current_pos)));
    
% if angle equals to zero, startpoint, current point, and destination    
% are on a line, which means that Roomba is on "m-line"
if angle == 0 
    b = 1;
else
    b = 0;

end

