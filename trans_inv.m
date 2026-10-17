function invT = trans_inv(T)
[R, p] = trans_to_Rp(T);
invT = [transpose(R), -transpose(R) * p; 0, 0, 0, 1];
end
