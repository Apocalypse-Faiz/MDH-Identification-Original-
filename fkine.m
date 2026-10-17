function T = fkine(M, Slist, q)
T = M;
for i = length(q): -1: 1
    T = matrix_exp6(vec_to_se3(Slist(:, i) * q(i))) * T;
end
end