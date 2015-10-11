function pose = turn_towards_dest(r, old_pose)
% turn the iCreate to make it head towards destination

global angle_tolerance
angle_tolerance = 3 * pi / 180; % TODO: what value to choose?

global dot_tolerance
dot_tolerance = 0.01; % DO NOT USE

global goal_coord
rob_vec = [old_pose(1, 1) old_pose(1, 2)];

dest_vec = goal_coord - [0 0];
dest_vec = dest_vec / norm(dest_vec);

display(rob_vec)
display(dest_vec)

dest_norm_vec = norm_vec(dest_vec);
display(dest_norm_vec)
display('yiiiiiiipeeeeeeeee')
display(dot(rob_vec, dest_vec))
display(dot(dest_norm_vec, rob_vec));

if dot(rob_vec, dest_vec) > 0.99 % no need to turn iCreate if it
                                         % already heads towards destnation
    pose = old_pose;

elseif dot(dest_norm_vec, rob_vec) < 0 % we need to make iCreate turn left

    angle_accum = AngleSensorRoomba(r);

    SetFwdVelRadiusRoomba(r, TURN_VEL, eps);
    while dot(rob_vec, dest_vec) < 0.999 % TODO: need to consider bump???
        pause(0.1)

        angle = AngleSensorRoomba(r);
        rob_vec = rot(rob_vec, angle);
        display(dot(rob_vec, dest_vec))

        angle_accum = angle_accum + angle;
        trplot2(old_pose * se(0, 0, angle_accum));
    end
    SetFwdVelRadiusRoomba(r, 0, inf);

    pose = se(0, 0, angle_accum + AngleSensorRoomba(r));
    pose = old_pose * pose;

else % current_angle > 0, we need to make iCreate turn right
    
    angle_accum = AngleSensorRoomba(r);

    SetFwdVelRadiusRoomba(r, TURN_VEL, -eps);
    while dot(rob_vec, dest_vec) < 0.999 % TODO: need to consider bump???
        pause(0.1)

        angle = AngleSensorRoomba(r);
        rob_vec = rot(rob_vec, angle);
        display(dot(rob_vec, dest_vec))

        angle_accum = angle_accum + angle;

    end
    SetFwdVelRadiusRoomba(r, 0, inf);

    pose = se(0, 0, angle_accum + AngleSensorRoomba(r));
    pose = old_pose * pose;
end

end

function new_v = rot(v, rad)
    new_v = v * [cos(rad) -sin(rad); sin(rad) cos(rad)];
end

function v = norm_vec(dv)
    v = [dv(2), -dv(1)];
end
