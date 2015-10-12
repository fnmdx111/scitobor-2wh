function mainloop(r)
%MAINLOOP Summary of this function goes here
%   Detailed explanation goes here
    global tolerance;
    tolerance = 0.25;

    global simulator
    simulator = 0;

    global circumnavigate_ok

    origin = se(0,0,0);
    endpose = se(4,0,0);

    global goal_coord
    goal_coord = pos_from_ht(endpose);
        
    trplot2(origin, 'color', 'g');
    hold on
    trplot2(endpose, 'color', 'r');
    hold on
   

    pose=se(DistanceSensorRoomba(r), 0, AngleSensorRoomba(r));
    
    while true
        pose=turn_towards_dest(r,pose);
        display(pose)

        %move forward until bump
        CALIBRATE_COUNTER = 0;
        counter = 1;

        bump=bump_test(r);
        while bump==NO_BUMP
            if counter > CALIBRATE_COUNTER
                display('calibrating-----------------------')
                pose = turn_towards_dest(r, pose);
                counter = 0;
            end
            counter = counter + 1;

            display(norm(pos_from_ht(pose) - goal_coord))

            dist = move_forward(r, WALK_VEL, WALK_TIME);
            pose = pose * se(dist, 0, 0);

            trplot2(pose);
            hold on

            if norm(pose(:, 3) - endpose(:,3)) < tolerance
                display('SUCEED')
                SetFwdVelRadiusRoomba(r, 0, inf);
                return;
            end
            bump = bump_test(r);
        end

        %circumnavigate
        %if arrive end point--exit
        %if arrive last bump point--exit,failure
        %if meet the intersected line, break and turn towards end point

        pose = circumnavigate(r, pose);
        if circumnavigate_ok == 0
        elseif circumnavigate_ok == 1
            display('SUCCEED')
            SetFwdVelRadiusRoomba(r, 0, inf);
            break;
        elseif circumnavigate_ok == -1
            display('FAIL')
            SetFwdVelRadiusRoomba(r, 0, inf);
            break;
        end
    end

end
