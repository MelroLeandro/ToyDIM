function G = Global(v)
% Author: Carlos Leandro
% Data: 25Fev16
% Version:

e0=v(1);
e1=v(2);
e2=v(3);
e3=v(4);

G=[-e1 e0 -e3 e2; -e2 e3 e0 -e1; -e3 -e2 e1 e0];

end

