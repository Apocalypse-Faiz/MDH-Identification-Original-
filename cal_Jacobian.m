function J = cal_Jacobian(Slist, q)
J = Slist;
T = eye(4);
for i = 2: length(q)
    T = T * matrix_exp6(vec_to_se3(Slist(:, i - 1) * q(i - 1)));
	J(:, i) = adjoint(T) * Slist(:, i);
end
end