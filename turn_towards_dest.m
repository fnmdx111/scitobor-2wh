function pose = turn_towards_dest(r, old_pose)
% turn the Roomba to make it head towards destination


current_angle = acos(old_pose(1, 1)); % pose(1, 1) = cos(theta)

global angle_tolerance; 
angle_tolerance = 0.01; % TODO: what value to choose?

if abs(current_angle) <= angle_tolerance % no need to turn Roomba if it
                                         % already heads towards destnation
    pose = old_pose;
    
 
else if current_angle < 0 % we need to make Roomba turn left
  
    angle_accum = AngleSensorRoomba(r);

    SetFwdVelRadiusRoomba(r, TURN_VEL, eps);
    while angle_accum < abs(current_angle) % TODO: need to consider bump???
        pause(0.2)

        angle_accum = angle_accum + AngleSensorRoomba(r); 

    end
    SetFwdVelRadiusRoomba(r, 0, inf);

    pose = se(0, 0, angle_accum);
    pose = old_pose * pose;
    
    
    else % current_angle > 0, we need to make Roomba turn right
    
    angle_accum = AngleSensorRoomba(r);

    SetFwdVelRadiusRoomba(r, TURN_VEL, -eps);
    while abs(angle_accum) < current_angle % TODO: need to consider bump???
        pause(0.2)

        angle_accum = angle_accum + AngleSensorRoomba(r); 

    end
    SetFwdVelRadiusRoomba(r, 0, inf);

    pose = se(0, 0, angle_accum);
    pose = old_pose * pose;
    end
end
    

end

