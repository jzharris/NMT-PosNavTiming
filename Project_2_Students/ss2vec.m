function [w] = ss2vec(Ohm)
% build vector w from skew-symmetric matrix Ohm
w = [Ohm(3,2); Ohm(1,3); Ohm(2,1)];
end