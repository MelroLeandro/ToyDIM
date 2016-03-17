function sk = skew(v)
% Author: Carlos Leandro
% Data: 25Fev16
% Version:

a1=v(1);
a2=v(2);
a3=v(3);

sk=[0 -a3 a2; a3 0 -a1; -a2 a1 0];

end

