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

function v = FRONT
    v = 1;
end

function v = LEFT
    v = 2;
end

function v = RIGHT
  v = 3;
end

function v = NO_BUMP
  v = 0;
end
