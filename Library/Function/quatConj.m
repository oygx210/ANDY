function p = quatConj(q)
%QUATMULTIPLY Multiply two quaternions.
%   P = QUATMULTIPLY(Q, R) multiplies quaternions Q and R, returning their
%   product P.

if size(q, 1) ~= 4
  error('Expecting quaternions as Column.');
end

p = [q(1);-q(2:4)];
end