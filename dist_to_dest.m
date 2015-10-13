function dist = dist_to_dest(old_pose)
    global goal_coord % [4 0] destination

    current_pos = pos_from_ht(old_pose);

    dist = norm(goal_coord - current_pos);
end

