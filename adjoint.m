function AdT = adjoint(T)
[R, p] = trans_to_Rp(T);
AdT = [R, zeros(3); vec_to_so3(p) * R, R];
end