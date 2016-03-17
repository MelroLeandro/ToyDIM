function D = dist(Plano, Esfera)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

p=(-Plano.vecn'*Plano.r)/norm(Plano.vecn);
n=Plano.vecn/norm(Plano.vecn);

D=n'*Esfera.r+p;

end

