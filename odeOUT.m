function Output = odeOUT(t,y,flag,varargin)
%odeOUT  Time series ODE output function.
% Autor: Carlos Leandro
% Data: 25Fev16
% Version:

global TT  Err Pe Ke
global World

Output=[];
if nargin < 3 || isempty(flag) 
    TT=[TT t];
    Err=[Err World.Err];
    Ke=[Ke World.Ke];
    Pe=[Pe World.Pe];
    World.Err=[];
else
    
    switch(flag)
      case 'init'
        TT=[];
        Err=[];
    end
end

end  % odempi


