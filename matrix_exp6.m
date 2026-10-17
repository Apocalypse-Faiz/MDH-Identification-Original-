function T = matrix_exp6(se3)
omgtheta = so3_to_vec(se3(1: 3, 1: 3));
if near_zero(norm(omgtheta))
    T = [eye(3), se3(1: 3, 4); 0, 0, 0, 1];
else
    [~, theta] = axis_angle(omgtheta);
    omgmat = se3(1: 3, 1: 3) / theta; 
    T = [matrix_exp3(se3(1: 3, 1: 3)), ...
         (eye(3) * theta + (1 - cos(theta)) * omgmat ...
          + (theta - sin(theta)) * omgmat * omgmat) ...
            * se3(1: 3, 4) / theta;
         0, 0, 0, 1];
end
end