% homogeneous coordinating
function pose = se(x, y, theta)
    pose = [cos(theta), -sin(theta), x;
            sin(theta), cos(theta), y;
            0, 0, 1];
end
