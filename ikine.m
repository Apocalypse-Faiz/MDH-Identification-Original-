function [q, success] = ikine(Slist, M, T, q0, eomg, ev)
q = q0;
i = 0;
max_iterations = 50;
Tsb = fkine(M, Slist, q);
Vs = adjoint(Tsb) * se3_to_vec(matrix_log6(trans_inv(Tsb) * T));
err = norm(Vs(1: 3)) > eomg || norm(Vs(4: 6)) > ev;
while err && i < max_iterations
    q = q + pinv(cal_Jacobian(Slist, q)) * Vs;
    i = i + 1;
    Tsb = fkine(M, Slist, q);
    Vs = adjoint(Tsb) * se3_to_vec(matrix_log6(trans_inv(Tsb) * T));
    err = norm(Vs(1: 3)) > eomg || norm(Vs(4: 6)) > ev;
end
success = ~ err;
end