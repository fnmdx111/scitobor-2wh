function mainloop(r)
%MAINLOOP Summary of this function goes here
%   Detailed explanation goes here
    global tolerance;
    tolerance = 0.2;
    origin = se(0,0,0);
    endpose = se(4,0,0);
    pose=se(DistanceSensorRoomba(r), 0, AngleSensorRoomba(r));
    i=1;
    
    while true
        %if robot has not faced to endpoint, turn towards endpoint
       while (pose(2,1)/pose(1,1))~= (4-pose(1,3))/(-pose(2,3))  
           pose=turn_towards_dest(r,pose);
       end
       
       %move forward until bump
       bump=bump_test(r);
       while bump==NO_BUMP
           dist = move_forward(r, WALK_VEL, WALK_TIME);
           pose(1,3) = pose(1,3)+dist*pose(1,1);
           pose(2,3) = pose(2,3)+dist*pose(2,1);
           if norm(pose(:, 3) - endpose(:,3)) < tolerance
               exit(0);
           end
           bump = bump_test(r);
       end
       bumppose=pose;
       
       %circumnavigate
       %if arrive end point--exit
       %if arrive last bump point--exit,failure
       %if meet the intersected line, break and turn towards end point
       while true
           pose=circumnavigate(r);
           if norm(pose(:, 3) - endpose(:,3)) < tolerance
                exit(0);
           end
           if norm(pose(:, 3) - bumppose(:,3)) < tolerance
               exit(0);
           end
           if(is_intersected)
               break
           end
       end
       
    end

end
