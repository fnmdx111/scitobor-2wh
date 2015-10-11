function b = is_intersected(old_pose)
% judege whether or not iCreate is on m-line
% return 1 if iCreate is on m-line
% return 0 otherwise

global goal_coord % [4 0] destination

current_pos = pos_from_ht(old_pose); % current position
display(current_pos)

tolerance = 0.075;

if abs(current_pos(2)) <= tolerance
    b = 1;
else
    b = 0;
end

end