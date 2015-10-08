function dist = move_forward(r, vel, time)
    init = DistanceSensorRoomba(r);
    SetFwdVelRadiusRoomba(r, vel, inf);
    pause(time)

    dist = DistanceSensorRoomba(r) + init;
end
