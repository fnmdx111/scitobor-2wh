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

