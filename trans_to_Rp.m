function [R, p] = trans_to_Rp(T)
R = T(1: 3, 1: 3);
p = T(1: 3, 4);
end