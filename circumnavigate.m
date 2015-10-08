%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS W4733 Computational Aspects of Robotics 2015
%
% Homework 1
%
% Team number: 22
% Team leader: Zihang Chen (zc2324)
% Team members: Yixing Chen (yc3094), Xin Yang (xy2290)
%   
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
%
function finish = am_i_done(r, pose, obstacle_to_goal_dist)
    global tolerance
    global goal_coord
    global obstacle_hit_pos

    current_pos = pos_from_ht(pose);
    dist = norm(goal_coord - current_pos);

    if dist < tolerance
        finish = 1;
    elseif norm(obstacle_hit_pos - current_pos) < tolerance
        finish = -1;
    elseif is_intersected(pose)
        if dist < obstacle_to_goal_dist
            if bump_test(r) == NO_BUMP
                finish = 0;
            end
        end
    else
        finish = 999;
    end
end

%% main function
% our code should work even when wall sensor does not work
function ok = circumnavigate(r, old_pose)
    global simulator
    simulator = 1;

    global goal_coord
    goal_coord = [4 0];

    global tolerance
    tolerance = 0.25; % this will make up the errors incurred in navigation

    origin = old_pose;
    pose = origin;

    global obstacle_hit_pos
    obstacle_hit_pos = pos_from_ht(pose);
    % we need this position every time we want to tell if the
    % robot is at the obstacle hit point again,
    % we also need the distance from obstacle to the goal, which
    % we only calculate once

    obstacle_to_goal_dist = norm(obstacle_hit_pos - origin);

    % ensure Create will not stop at first few steps 
    while norm(pose(:, 3) - origin(:, 3)) <= tolerance
        pose = next_move(r, pose); % update position
    end

    % move before Create comes back to the point where it first
    % hit the wall
    f = am_i_done(r, pose, obstacle_to_goal_dist);
    while f == 999
        pose = next_move(r, pose); % update position
        f = am_i_done(r, pose, obstacle_to_goal_dist);
    end

    ok = f;
    % if ok == 0, forward we go
    % if ok == 1, goal we are at
    % if ok == -1, trapped we must be
    % else (f == 999), go on circumnavigation
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
            pose = old_pose * bypass(r);
        end
    else
        if wall == 1
            pose = old_pose * walk_straightly(r);
        else
            pose = old_pose * turn_right_until_a_wall(r);
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

function pose = turn_right_until_a_wall(r)
    global simulator

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
    if simulator == 0 % Create tends to turn more than we want it
        max_angle = 17 * pi / 180;
    end

    delta_pose = se(0., 0., 0.);

    max_angle_reached = 0;

    while wall == 0 && bump == NO_BUMP % if we bumped into something
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
            while dist_accum < BYPASS_DIST * 0.5 % walk ahead
                % TODO: Maybe there will be a case that we stop here?
                % FIXED: Every time we walked a bit ahead, the distance
                %        will be recalculated again, so no need to test
                %        the distance here.
                dist_delta = move_forward(r, WALK_VEL, WALK_TIME);
                dist_accum = dist_accum + dist_delta;
            end
            delta_pose = delta_pose *...
                         se(dist_accum, 0.,...
                            angle_accum + AngleSensorRoomba(r));
                            % Accumulate the angle every time!
                            % Accumulate the angle every time!
                            % Accumulate the angle every time!
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

function new_pose = bypass(r)
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
    end
    new_pose = new_pose * se(dist_accum, 0., AngleSensorRoomba(r));
                                             % Accumulate the angle
                                             % every time!
end
