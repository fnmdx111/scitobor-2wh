%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Members: Zihang Chen (zc2324) %
%          Yixing Chen (yc3094) %
%          Xin Yang    (xy2290) %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% This file is concatenated automatically by <concat.py>.

% This function is generated automatically by <concat.py>.
function hw2_team_22(r)
    global simulator
    simulator = 1;

    mainloop(r);
end

% <concat.py>: concatenating mainloop.m ------->
function mainloop(r)
    global tolerance;
    tolerance = 0.25;

    global simulator

    global circumnavigate_ok

    origin = se(0,0,0);
    endpose = se(4,0,0);

    global goal_coord
    goal_coord = pos_from_ht(endpose);



    pose=se(DistanceSensorRoomba(r), 0, AngleSensorRoomba(r));

    while true
        pose=turn_towards_dest(r,pose);

        CALIBRATE_COUNTER = 0;
        counter = 2;

        %move forward until bump
        bump=bump_test(r);
        while bump==NO_BUMP
            if counter > CALIBRATE_COUNTER
                % Because it cannot be guaranteed that we exit circumnavigation
                % mode with perfect orientation towards goal. And a small error
                % in orientation most often turns out to be disastrous. So the
                % orientation must be calibrated before it's too late.
                % We calibrate our orientation every 2 steps.
                pose = turn_towards_dest(r, pose);
                counter = 0;
            end
            counter = counter + 1;


            dist = move_forward(r, WALK_VEL, WALK_TIME);
            pose = pose * se(dist, 0, 0);


            if norm(pose(:, 3) - endpose(:,3)) < tolerance
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
        if circumnavigate_ok == 0 % We finished circumnavigation, and need to
                                  % go forward, so do nothing here
        elseif circumnavigate_ok == 1
            % Remember to stop the robot
            SetFwdVelRadiusRoomba(r, 0, inf);
            break;
        elseif circumnavigate_ok == -1
            % Same here
            SetFwdVelRadiusRoomba(r, 0, inf);
            break;
        end
    end
end

% <concat.py>: concatenating bump_test.m ------->
function bump = bump_test(r)
    [br, bl, ~, ~, ~, bf] = BumpsWheelDropsSensorsRoomba(r);

    if bl == 1
        bump = LEFT;
    elseif br == 1
        bump = RIGHT;
    elseif bf == 1
        bump = FRONT;
    else
        bump = NO_BUMP;
    end
end

% <concat.py>: concatenating BYPASS_DIST.m ------->
function h = BYPASS_DIST
% BYPASS_DIST controls how long by walking forward
% the robot bypasses current obstacle

% If this value is too much, we are too far away from
% the obstacle, and stand little chance of getting
% back to the obstacle (because we're circumnavigating).

% And if it's too little, the robot won't move at all
% due to frictions.

h = 0.05;
end


% <concat.py>: concatenating circumnavigate.m ------->
% `circumnavigate` will work even without wall sensor
function pose = circumnavigate(r, old_pose)
    global circumnavigate_ok
    circumnavigate_ok = 999;

    global tolerance

    global goal_coord

    origin = old_pose;
    pose = origin;

    global obstacle_hit_pos
    obstacle_hit_pos = pos_from_ht(pose);
    % we need this position every time we want to tell if the
    % robot is at the obstacle hit point again,
    % we also need the distance from obstacle to the goal, which
    % we only calculate once

    global obstacle_to_goal_dist
    obstacle_to_goal_dist = norm(obstacle_hit_pos - goal_coord);

    % ensure Create will not stop at first few steps
    while norm(pose(:, 3) - origin(:, 3)) <= tolerance
        pose = next_move(r, pose); % update position
    end

    % move before Create comes back to the point where it first
    % hit the wall
    f = am_i_done(r, pose);
    while f == 999
        pose = next_move(r, pose); % update position


        f = am_i_done(r, pose);
        if f ~= 999
            break
        end
    end

    circumnavigate_ok = f;
    % if ok == 0, forward we go
    % if ok == 1, goal we are at
    % if ok == -1, trapped we must be
    % else (f == 999), go on circumnavigation
end

function finish = am_i_done(r, pose)
    trap_tolerance = 0.15;
    global tolerance
    global goal_coord
    global obstacle_hit_pos
    global obstacle_to_goal_dist

    current_pos = pos_from_ht(pose);
    dist = norm(goal_coord - current_pos);

    if dist < tolerance
        finish = 1;
        return
    elseif norm(obstacle_hit_pos - current_pos) < trap_tolerance
        finish = -1; % we are back at where we start circumnavigating
        return       % we are most certainly trapped
    elseif is_intersected(pose) == 1
        if dist < obstacle_to_goal_dist
            finish = 0;
            return;
        end
    end
    finish = 999;
end

% move, and update the position of Create
function pose = next_move(r, old_pose)
    % the central idea is that
    % * we walk along the wall if there is a wall,
    % * we stop at bumps and bypass them by turning left and moving forward
    %     for a short distance,
    % * we turn right and move forward for a short distance if we can't
    %     find a wall.

    wall = wall_test(r);
    bump = bump_test(r);
    if bump ~= NO_BUMP   % if Create bumps into a something, it must stop
        if bump == FRONT
            pose = old_pose * turn_left_till_bump_gone(r);
        else
            pose = old_pose * bypass(r, old_pose);
        end
    else
        if wall == 1
            pose = old_pose * walk_straightly(r);
        else
            pose = old_pose * turn_right_until_a_wall(r, old_pose);
        end
    end
end

function pose = turn_left_till_bump_gone(r)
    bump = bump_test(r);

    angle_accum = AngleSensorRoomba(r);
    % As a matter of fact, one can only accumulate angles relative to
    % prior movements, instead of relative to the point when it just
    % begins to turn (you'll miss the angles ghostly turned during the
    % last movement).

    % These values that were discarded earlier play an important role in
    % producing errors during navigation.

    SetFwdVelRadiusRoomba(r, TURN_VEL, eps);
    while bump ~= NO_BUMP
        pause(0.2)

        angle_accum = angle_accum + AngleSensorRoomba(r);

        bump = bump_test(r);
    end
    SetFwdVelRadiusRoomba(r, 0, inf);

    pose = se(0, 0, angle_accum);
end

function pose = turn_right_until_a_wall(r, old_pose)
    global simulator
    global circumnavigate_ok

    angle_accum = AngleSensorRoomba(r);

    wall = wall_test(r);
    bump = bump_test(r);

    max_angle = 33 * pi / 180; % a threshold which controls the
                               % maximum turning angle that ensures
                               % that Create will not end up being in
                               % the wrong direction

                               % This value shouldn't be too large:
                               %   it would lead Create to the wrong
                               %   direction.
                               % Nor should it be too small:
                               %   the distance error incurred in turning
                               %   right will be unacceptible.
    if simulator == 0 % Create tends to turn more than we want it to
        max_angle = 11 * pi / 180;
    end

    delta_pose = se(0., 0., 0.);

    max_angle_reached = 0;

    while (wall == 0) && (bump == NO_BUMP) % if we bumped into something
                                           % we most certainly shouldn't move
        SetFwdVelRadiusRoomba(r, TURN_VEL, -eps);

        pause(0.2)

        angle = AngleSensorRoomba(r);
        angle_accum = angle_accum + angle;

        if max_angle < abs(angle_accum)
            % If we have turned by this angle,
            % just stop turning, because there
            % is a big chance that we won't find
            % the next wall if we continue to turn,
            % so we instead walk ahead a few steps,
            % and then start turning again.
            dist_accum = DistanceSensorRoomba(r);
            dist_accum = DistanceSensorRoomba(r); % TODO!!!

            while dist_accum < BYPASS_DIST % walk ahead
                % TODO: Maybe there will be a case that we stop here?
                % FIXED: Every time we walked a bit ahead, the distance
                %        will be recalculated again, so no need to test
                %        the distance here.
                dist_delta = move_forward(r, WALK_VEL, WALK_TIME);
                dist_accum = dist_accum + dist_delta;

                circumnavigate_ok = am_i_done(...
                    r,...
                    old_pose * se(dist_accum, 0, angle_accum));

                if circumnavigate_ok ~= 999
                    pose = se(dist_accum, 0, angle_accum + AngleSensorRoomba(r));
                    return
                end

                bump_ = bump_test(r);
                if bump_ ~= NO_BUMP
                    break;
                end
            end
            delta_pose = delta_pose *...
                         se(dist_accum, 0.,...
                            angle_accum + AngleSensorRoomba(r));

            max_angle_reached = 1;
            break
        end

        wall = wall_test(r);
        bump = bump_test(r);
    end
    SetFwdVelRadiusRoomba(r, 0, inf);


    if max_angle_reached == 1
        angle_accum = 0.;
        % if maximum angle was reached, there's no need in
        % accumulating the angle to the pose again
    end

    pose =  delta_pose * se(0.,...
                            0.,...
                            angle_accum + AngleSensorRoomba(r));
end

function new_pose = bypass(r, old_pose)
    global circumnavigate_ok

    angle_accum = AngleSensorRoomba(r);

    bump = bump_test(r);

    % turn left until bumps are gone
    SetFwdVelRadiusRoomba(r, TURN_VEL, eps);
    while bump ~= NO_BUMP
        pause(0.2)

        angle_accum = angle_accum + AngleSensorRoomba(r);

        bump = bump_test(r);
    end
    SetFwdVelRadiusRoomba(r, 0, inf);

    new_pose = se(0., 0., angle_accum);

    dist_accum = 0.;
    while dist_accum < BYPASS_DIST % walk ahead
        dist_delta = move_forward(r, WALK_VEL, WALK_TIME);
        dist_accum = dist_accum + dist_delta;

        circumnavigate_ok = am_i_done(...
            r,...
            old_pose * se(dist_accum, 0, angle_accum));
        if circumnavigate_ok ~= 999
            new_pose = new_pose * se(dist_accum, 0., AngleSensorRoomba(r));
            return
        end

        bump_ = bump_test(r);
        if bump_ ~= NO_BUMP
            break;
        end
    end
    new_pose = new_pose * se(dist_accum, 0., AngleSensorRoomba(r));
                                             % Accumulate the angle
                                             % every time!
end

% <concat.py>: concatenating dist_to_dest.m ------->
function dist = dist_to_dest(old_pose)
    global goal_coord % [4 0] destination

    current_pos = pos_from_ht(old_pose);

    dist = norm(goal_coord - current_pos);
end


% <concat.py>: concatenating FRONT.m ------->

function v = FRONT
    v = 1;
end

% <concat.py>: concatenating is_intersected.m ------->
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

    tolerance = 0.075;

    if abs(current_pos(2)) <= tolerance
        b = 1;
    else
        b = 0;
    end

end

% <concat.py>: concatenating LEFT.m ------->

function v = LEFT
    v = 2;
end

% <concat.py>: concatenating move_forward.m ------->
function dist = move_forward(r, vel, time)
    init = DistanceSensorRoomba(r);
    SetFwdVelRadiusRoomba(r, vel, inf);
    pause(time)

    dist = DistanceSensorRoomba(r) + init;
end

% <concat.py>: concatenating NO_BUMP.m ------->

function v = NO_BUMP
  v = 0;
end

% <concat.py>: concatenating pos_from_ht.m ------->
function v = pos_from_ht(ht)
%POS_FROM_HT Summary of this function goes here
%   Detailed explanation goes here
    v = [ht(1, 3) ht(2, 3)];

end


% <concat.py>: concatenating RIGHT.m ------->
function v = RIGHT
  v = 3;
end

% <concat.py>: concatenating se.m ------->
% homogeneous coordinating
function pose = se(x, y, theta)
    pose = [cos(theta), -sin(theta), x;
            sin(theta), cos(theta), y;
            0, 0, 1];
end

% <concat.py>: concatenating turn_towards_dest.m ------->
function pose = turn_towards_dest(r, old_pose)
% turn the iCreate to make it head towards destination

% The basic idea here is that we calculate the dot product between
% the unit vector from start (origin) to goal, and the unit vector
% of current orientation.
% If the dot product result is nearly one, we are aligned to the
% goal;
% otherwise, turn a little and calculate the dot product again

% A little optimization here is that using the one of the normal
% vectors of the destination vector (from start to goal), we can
% minimize the angle we turn. If dot product between the orientation
% vector and the selected normal vector is bigger than 0, then we
% know that by turning right, the angle we turn will be less than
% 180 degrees and vice versa.
% The trick here is to test out the corresponding normal vector and
% the orientation we turn to.

global goal_coord
rob_vec = [old_pose(1, 1) old_pose(1, 2)];

dest_vec = goal_coord - [0 0];
dest_vec = dest_vec / norm(dest_vec);


dest_norm_vec = norm_vec(dest_vec);

if dot(rob_vec, dest_vec) > 0.999 % no need to turn iCreate if it
                                         % already heads towards destnation
    pose = old_pose;

elseif dot(dest_norm_vec, rob_vec) < 0 % we need to make iCreate turn left

    angle_accum = AngleSensorRoomba(r);

    SetFwdVelRadiusRoomba(r, TURN_VEL, eps);
    while dot(rob_vec, dest_vec) < 0.998 % TODO: need to consider bump???
        pause(0.1)

        angle = AngleSensorRoomba(r);
        rob_vec = rot(rob_vec, angle);

        angle_accum = angle_accum + angle;
    end
    SetFwdVelRadiusRoomba(r, 0, inf);

    pose = se(0, 0, angle_accum + AngleSensorRoomba(r));
    pose = old_pose * pose;

else % current_angle > 0, we need to make iCreate turn right

    angle_accum = AngleSensorRoomba(r);

    SetFwdVelRadiusRoomba(r, TURN_VEL, -eps);
    while dot(rob_vec, dest_vec) < 0.998 % TODO: need to consider bump???
        pause(0.1)

        angle = AngleSensorRoomba(r);
        rob_vec = rot(rob_vec, angle);

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

% <concat.py>: concatenating TURN_VEL.m ------->
function h = TURN_VEL
%TURN_VEL Summary of this function goes here
%   Detailed explanation goes here

    global simulator

    if simulator
        h = 0.001; % if we set h = 0.1 on the simulator, it just blows up
    else
        h = 0.05;
    end
end


% <concat.py>: concatenating walk_straightly.m ------->
function pose = walk_straightly(r)
    dist_accum = 0.;

    pose = se(0, 0, AngleSensorRoomba(r));
    % better include this angle

    dist_accum = dist_accum + move_forward(r, WALK_VEL, WALK_TIME);

    pose = pose * se(dist_accum, 0, AngleSensorRoomba(r));
end

% <concat.py>: concatenating WALK_TIME.m ------->
function h = WALK_TIME
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

  h = 0.2;
end


% <concat.py>: concatenating WALK_VEL.m ------->
function h = WALK_VEL
%WALK_VEL Summary of this function goes here
%   Detailed explanation goes here

h = 0.2;
end

% <concat.py>: concatenating wall_test.m ------->
function wall = wall_test(r)
    wall = WallSensorReadRoomba(r);
end

