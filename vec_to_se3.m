function se3 = vec_to_se3(V)
se3 = [vec_to_so3(V(1: 3)), V(4: 6); 0, 0, 0, 0];
end