function dist = dist_to_dest(old_pose)
% calculate distance to destination

global goal_coord % [4 0] destination

current_pos = pos_from_ht(old_pose); % current position

dist = norm(goal_coord - current_pos); % distance

end

