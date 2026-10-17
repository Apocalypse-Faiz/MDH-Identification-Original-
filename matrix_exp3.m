function  R = matrix_exp3(so3)
omgtheta = so3_to_vec(so3);
if near_zero(norm(omgtheta))
    R = eye(3);
else
    [~, theta] = axis_angle(omgtheta);
    omgmat = so3 / theta;
    R = eye(3) + sin(theta) * omgmat + (1 - cos(theta)) * omgmat * omgmat;
end
end