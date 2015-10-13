function b = is_intersected(old_pose)
% return 1 if iCreate is on m-line
% return 0 otherwise

    global goal_coord % [4 0] destination
    % We are solving a linear equation here
    % the point we start and the goal point forms a line,
    % in this particular case, it's y = 0,
    % So it satisfies that whenever the y value of our
    % coordinate is y (of course you don't do `equal float`),
    % we are on m-line.

    current_pos = pos_from_ht(old_pose);
    display(current_pos)

    tolerance = 0.075;

    if abs(current_pos(2)) <= tolerance
        b = 1;
    else
        b = 0;
    end

end