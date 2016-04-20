function Output = odeOUT(t,y,flag,varargin)
%odeOUT  Time series ODE output function.
% Author: Carlos Leandro
% Data: 25Fev16
% Version:

global TT  Err Pe Ke
global World
global BodyList % List with body identifiers
global Bodies   % Structure with every rigid bodies in the system
global Simulation

Output=[];
if nargin < 3 || isempty(flag) 
    TT=[TT t];
    Err=[Err World.Err];
    Ke=[Ke World.Ke];
    Pe=[Pe World.Pe];
    World.Err=[];
    for indexE = 1:World.nbodies
        BodyName=BodyList{indexE};
        Simulation.(BodyName).vel=[Simulation.(BodyName).vel Bodies.(BodyName).r_d];
        Simulation.(BodyName).acc=[Simulation.(BodyName).acc Bodies.(BodyName).r_dd];
        for indexP=1:length(Bodies.(BodyName).PointsList)
            pointname=Bodies.(BodyName).PointsList{indexP};

            Simulation.(BodyName).(pointname)=[ Simulation.(BodyName).(pointname) Bodies.(BodyName).Points.(pointname).rP];
        end
                
        if Bodies.(BodyName).flexible % have a flexible part

            for  index1 = 1:Bodies.(BodyName).NumberNodes
               NodeName = Bodies.(BodyName).NodeList(index1,:);
               Simulation.(BodyName).(NodeName)= [Simulation.(BodyName).(NodeName) Bodies.(BodyName).node.(NodeName).rP];
            end
        end

    end
else
    
    switch(flag)
      case 'init'
        TT=[];
        Err=[];
        
        
        for indexE = 1:World.nbodies
            BodyName=BodyList{indexE};
            %Simulation.(BodyName).u_d=[];
            Simulation.(BodyName).vel=[];
            Simulation.(BodyName).acc=[];
        
            for indexP=1:length(Bodies.(BodyName).PointsList)
                pointname=Bodies.(BodyName).PointsList{indexP};

                Simulation.(BodyName).(pointname)= [];
                Simulation.(BodyName).(pointname)=Bodies.(BodyName).Points.(pointname).rP;
            end

            if Bodies.(BodyName).flexible % have a flexible part

                for  index1 = 1:Bodies.(BodyName).NumberNodes
                   NodeName = Bodies.(BodyName).NodeList(index1,:);
                   Simulation.(BodyName).(NodeName)= [];
                end
            end
        end
    end
end

end  


