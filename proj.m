function P = proj(n)
% Author: Carlos Leandro
% Data: 25Fev16
% Version:

A=n(1);
B=n(2);
C=n(3);

P=[1-A^2, -A*B, -A*C; -A*B, 1-B^2, -B*C; -A*C, -B*C, 1-C^2];

end

