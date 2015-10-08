function pose = walk_straightly(r)
    dist_accum = 0.;

    pose = se(0, 0, AngleSensorRoomba(r));
    % better include this angle

    dist_accum = dist_accum + move_forward(r, WALK_VEL, WALK_TIME);

    pose = pose * se(dist_accum, 0, AngleSensorRoomba(r));
end
