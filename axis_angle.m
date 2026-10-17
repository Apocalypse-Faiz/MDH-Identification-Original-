function [axis, angle] = axis_angle(vec)
angle = norm(vec);
axis = vec / angle;
end