%-------------------------------------------------------------------------%
%------------------------------Multibody Dynamic--------------------------%
%------------------------------A toy system-------------------------------% 
% Problem:  3 rods linked using two revolute joints 3D
% Autor: Carlos Leandro
% Data: 25Fev16
% Version: 
%-------------------------------------------------------------------------%
clc
clear all
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Glabal variables used to 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global World    % used to describe the world 
global BodyList % List with body identifiers
global JointList % List with dynamic constrains identifiers
global Bodies   % Structure with every rigid bodies in the system
global Joints   % Structure with dynamis constrains 


global TT Err Ke Pe

%%
% World parameters

World.g=[0;0;0.0];       % force to be applied on each body in each iteration

% Contact
World.n=1.5;
World.e=0.6;
World.k=2*10^4;
World.mu=0.74;

% Baumgarte Method
World.alpha=5.0;
World.beta=5.0;

% Debugging information

World.C1=[];
World.Err=[];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Multibody system configuration:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

BodyList={'C1'}; % list of system bodies
%JointList={'Fix'}; % list of dynamic constrains
JointList={};

NumBodies= length(BodyList);
%%
% Geometry: 'C1' is a flexible body

bodyName='C1';
Bodies.(bodyName).flexible=true;

count=1;
for i=1:4
    for j=1:4
        
        node=sprintf('C%dx%d',i,j);  
        
        Bodies.(bodyName).node.(node).idx = count;          % nodes indexation
        
        count=count+1;                          % nodes count
        
        % system masses and geometry
        Bodies.(bodyName).node.(node).m=1;      % node mass
        Bodies.(bodyName).node.(node).mu=1;      % local rot inertia
        Bodies.(bodyName).node.(node).charge=0; % node chage
        Bodies.(bodyName).node.(node).x=[0,i,j]';  % initial global positions
        Bodies.(bodyName).node.(node).w=[0,0,0]';  % initial hlobal orientation
        Bodies.(bodyName).node.(node).deltaP=[0,0,0]';  % relative displacement
        Bodies.(bodyName).node.(node).deltaP_d=[0,0,0]';
        Bodies.(bodyName).node.(node).omegaP=[0,0,0]';  % nodal amgular relative velocity
        Bodies.(bodyName).node.(node).omegaP_d=[0,0,0]';
        % node lattice external forces
        Bodies.(bodyName).node.(node).f=[0,0,0]'; % linear
        Bodies.(bodyName).node.(node).n=[0,0,0]'; % rotational
    end
end

%%
% System topology

Bodies.(bodyName).node.C1x1.Nei={'C1x2','C2x1'};
Bodies.(bodyName).node.C1x1.num=2;
Bodies.(bodyName).node.C1x2.Nei={'C1x1','C1x3','C2x2'};
Bodies.(bodyName).node.C1x2.num=3;
Bodies.(bodyName).node.C1x3.Nei={'C1x2','C1x4','C2x3'};
Bodies.(bodyName).node.C1x3.num=3;
Bodies.(bodyName).node.C1x4.Nei={'C1x3','C2x4'};
Bodies.(bodyName).node.C1x4.num=2;
Bodies.(bodyName).node.C2x1.Nei={'C1x1','C2x2','C3x1'};
Bodies.(bodyName).node.C2x1.num=3;
Bodies.(bodyName).node.C2x2.Nei={'C1x2','C2x1','C2x3','C3x2'};
Bodies.(bodyName).node.C2x2.num=4;
Bodies.(bodyName).node.C2x3.Nei={'C1x3','C2x2','C2x4','C3x3'};
Bodies.(bodyName).node.C2x3.num=4;
Bodies.(bodyName).node.C2x4.Nei={'C1x4','C2x3','C3x4'};
Bodies.(bodyName).node.C2x4.num=3;
Bodies.(bodyName).node.C3x1.Nei={'C2x1','C3x2','C4x1'};
Bodies.(bodyName).node.C3x1.num=3;
Bodies.(bodyName).node.C3x2.Nei={'C2x2','C3x1','C3x3','C4x2'};
Bodies.(bodyName).node.C3x2.num=4;
Bodies.(bodyName).node.C3x3.Nei={'C2x3','C3x2','C3x4','C4x3'};
Bodies.(bodyName).node.C3x3.num=4;
Bodies.(bodyName).node.C3x4.Nei={'C2x4','C3x3','C4x4'};
Bodies.(bodyName).node.C3x4.num=3;
Bodies.(bodyName).node.C4x1.Nei={'C3x1','C4x2'};
Bodies.(bodyName).node.C4x1.num=2;
Bodies.(bodyName).node.C4x2.Nei={'C3x2','C4x1','C4x3'};
Bodies.(bodyName).node.C4x2.num=3;
Bodies.(bodyName).node.C4x3.Nei={'C3x3','C4x2','C4x4'};
Bodies.(bodyName).node.C4x3.num=3;
Bodies.(bodyName).node.C4x4.Nei={'C3x4','C4x3'};
Bodies.(bodyName).node.C4x4.num=2;
%%
% Contact matrix


%%
% System center of mass and the moment of inertia tensor


for index=1:length(BodyList)
    bodyName=BodyList{index};
    
    v=[0,0,0]';
    m=0;
    
    if Bodies.(bodyName).flexible % is a flexible body
        
        NodeList=fieldnames(Bodies.(bodyName).node);
        
        for index1=1:length(NodeList)
            
            NodeName=NodeList{index1};
            
            bm = Bodies.(bodyName).node.(NodeName).m;
            x0 = Bodies.(bodyName).node.(NodeName).x;

            v = v + bm*x0;  
            m=m+bm;
        end
        
        Bodies.(bodyName).geo.m=m; % System mass
        
        Bodies.(bodyName).center_mass=v/m; % System center of mass
    
        % We assume here that World.center_mass=(0,0,0)
        I=eye(3);
        JP=zeros(3);

        for index1=1:length(NodeList)

            NodeName=NodeList{index1};
            
            x0 = (Bodies.(bodyName).node.(NodeName).x-Bodies.(bodyName).center_mass);
            
            Bodies.(bodyName).node.(NodeName).xP=x0;

            JP = JP + bm*(x0'*x0*I-x0*x0'); 

        end
        
        % moment of inertia tensor
        [Bodies.(bodyName).V, Bodies.(bodyName).geo.JP] = eig(JP); % V'*Jp*V is diagonal
        
       for  index1 = 1:length(NodeList)

            NodeName = NodeList{index1};

            Bodies.(bodyName).node.(NodeName).rP = Bodies.(bodyName).V'*Bodies.(bodyName).node.(NodeName).xP;  
       end
    end % is a flexible body
end

%%
%  C1  : frame fixed in space at the center of mass of all 
%        the representative atoms and oriented along the 
%        principal moments of inertia

Bodies.C1.NodeList   = fieldnames(Bodies.(bodyName).node);
Bodies.C1.NumberNodes = length(Bodies.C1.NodeList);

% Points in body C1
Bodies.C1.PointsList    = {'P1'};
Bodies.C1.Points.P1.sPp = [0,0,0]'; % local frame coordenates

% Vectors in body C1
Bodies.C1.VectorsList   = {'V1'};
Bodies.C1.Vectors.V1.sP = [1,0,0]'; % local frame coordenates

% Body initial values
Bodies.C1.r   = Bodies.C1.center_mass;
Bodies.C1.r_d = [0,0,0]';  % initial velocity
Bodies.C1.r_dd= [0,0,0]'; % initial acceleration
Bodies.C1.p   = [1,0,0,0]';  % initial euler parameters
Bodies.C1.p_d = [0,0,0,0]';% derivada dos parametros de Euler
Bodies.C1.w   = [0,0,0]';    % initial angular velocity
Bodies.C1.wp  = [0,0,0]';   % initial angular aceleration
Bodies.C1.np  = [0,0,0]';   % initial momento

% Matrial properties
Bodies.C1.stiffness = 1;

%%
% Contact matrix K; The global stiffness matrix G;
NNodes=0; % count number of nodes in the system

for idx=1:length(BodyList)
    bodyName=BodyList{idx};
   
    
    if Bodies.(bodyName).flexible
        
        Bodies.(bodyName).K = zeros(Bodies.(bodyName).NumberNodes);
        Bodies.(bodyName).G = zeros(6*Bodies.(bodyName).NumberNodes+6);
  
        NNodes = NNodes + Bodies.(bodyName).NumberNodes;
        
        for  index1=1:Bodies.(bodyName).NumberNodes

            NodeName = Bodies.(bodyName).NodeList{index1};

            Gii = zeros(3);
            Wii = zeros(3);
            
            i   = Bodies.(bodyName).node.(NodeName).idx; 
            delta_i = Bodies.(bodyName).node.(NodeName).xP; % equilibrium positions
            omega_i = Bodies.(bodyName).node.(NodeName).w; % equilibrium orientation
            Bodies.(bodyName).K(i,i)=1;
            for idx2=1:Bodies.(bodyName).node.(NodeName).num
                NodeName2=Bodies.(bodyName).node.(NodeName).Nei{idx2};
                j   = Bodies.(bodyName).node.(NodeName2).idx;
                delta_j = Bodies.(bodyName).node.(NodeName2).xP; % equilibrium positions
                omega_j = Bodies.(bodyName).node.(NodeName2).w; % equilibrium orientation

                dr = delta_i-delta_j;
                dp = omega_i-omega_j;

                Bodies.(bodyName).K(i,j)=1;

                Gij = Bodies.(bodyName).stiffness*(dr*dr')/(dr'*dr);
                Wij = zeros(3);

                Gii = Gii + Gij;
                Wii = Wii + Wij;

                Bodies.(bodyName).G(1+6*(i-1)+5:6+6*(i-1)+5,1+6*(j-1)+5:6+6*(j-1)+5)=[-Gij zeros(3,3); zeros(3,3) -Wij];
            end
            
            Bodies.(bodyName).G(1+6*(i-1)+5:6+6*(i-1)+5,1+6*(i-1)+5:6+6*(i-1)+5)=[Gii zeros(3,3); zeros(3,3) Wii];
        end
    end % body is flexible
end

%%
% System dynamic constrains

%Ground joint
Joints.Fix.type   = 'Fix';  % fix body in the space
Joints.Fix.body_1 = 'C1';   % body identidier

% Multibody system information

World.nbodies = length(BodyList);  % number of bodies in the system 
World.njoints = length(JointList); % number of joints in the system
World.NNodes  = NNodes;            % count number of nodes in the system
World.Msize   = World.NNodes*6+World.nbodies*6; % mass matrix size
%%
% The global mass matrix 


y_d=[];

World.M=zeros(World.Msize); % Flexible body mass matrix aluccation

count = 1;
for indexE = 1:World.nbodies
    BodyName=BodyList{indexE};
    Bodies.(BodyName).g = World.g;
    O = zeros(3,3);
    
    index = (count-1)*6+1;
    count = count+1;
    
    Bodies.(BodyName).index = index; % body index in the mass matrix
    
    % recursive definition for the initial force
    y_d=[y_d; Bodies.(BodyName).r;Bodies.(BodyName).p;Bodies.(BodyName).r_d;Bodies.(BodyName).w];
    if Bodies.(BodyName).flexible % have a flexible part
        
        for  index1 = 1:Bodies.(bodyName).NumberNodes
           NodeName = Bodies.(bodyName).NodeList{index1};
           
           deltaP   = Bodies.(bodyName).node.(NodeName).deltaP;   % equilibrium positions
           omegaP   = Bodies.(bodyName).node.(NodeName).omegaP;   % equilibrium orientation
           deltaP_d = Bodies.(bodyName).node.(NodeName).deltaP_d; % 
           omegaP_d = Bodies.(bodyName).node.(NodeName).omegaP_d; % 

           y_d=[y_d; deltaP; omegaP; deltaP_d; omegaP_d];
           
           index = (count-1)*6+1;
           count = count+1;
           Bodies.(bodyName).node.(NodeName).y_index=index;
        end
    end % have a flexible part
end

%% 
% Start simulation

% set integration parametters
t0   = 0;    % Initial time
t    = 10.00; % Final time
step = 0.01; % Time-step


tspan = [t0:step:t];

% Set integrator and its parametters

fprintf('\n\n ODE45\n\n')
tol = 1e-5;
options = odeset('RelTol',tol,'Stats','on','OutputFcn',@odeOUT); 

tic
[T, yT] = ode45(@updateaccel, tspan, y_d, options);
timeode45 = toc

%%
% Graphic output

fig = figure;
for index = 1:World.nbodies
    BodyName = BodyList{index};
    y = Bodies.(BodyName).V*yT(:,(index-1)*13+1:(index-1)*13+3)';
    plot(y(2,:), y(3,:))
    
    hold on
end
xlabel('y'),ylabel('z')
legend('position y','position z')
% print(fig,'Position y z','-dpng')
hold off

fig = figure;
for index = 1:World.nbodies
    BodyName = BodyList{index};
    y = Bodies.(BodyName).V*yT(:,(index-1)*13+1:(index-1)*13+3)';
    plot(y(1,:), y(3,:))  
    hold on
end
xlabel('x'),ylabel('z')
legend('position x','position z')
% print(fig,'Position x z','-dpng')
hold off

fig=figure;
for idx=1:World.nbodies
    BodyName=BodyList{idx};
    index = Bodies.(BodyName).index;
    
    y=Bodies.(BodyName).V*yT(:,index:index+2)';
    
    plot(y(1,:), y(2,:), 'r') 
    
    hold on
    if  Bodies.(BodyName).flexible
              
        for  index1=1:Bodies.(BodyName).NumberNodes
           NodeName=Bodies.(BodyName).NodeList{index1};
           
           y_node= Bodies.(BodyName).node.(NodeName).y_index;
           
           z=yT(:,y_node:y_node+2)';
           
           plot(z(1,:), z(2,:), 'b') 
           
        end
    end
end
xlabel('x'),ylabel('y')
legend('position x','position y')
% print(fig,'Position x y','-dpng')
hold off

% Velocidades
fig=figure;
for indexE=1:World.nbodies %ciclo for actua sobre todos os diferentes esferas
    
    
    L=yT(:,(indexE-1)*13+8:(indexE-1)*13+10);
    v=[];
    for i=1:length(L)
        v=[v norm(L(i,:))];
    end

    plot(T, v) 
    hold on
end
xlabel('T'),ylabel('x[m/s]')
%print(fig,'velocity x','-dpng')
hold off

% Error on thr system constrains
fig=figure;
plot( Err) 
xlabel('iteration'),ylabel('Error')

% System Energy
fig=figure;
plot(Ke) 
hold on
plot(Pe)
xlabel('iteration'),ylabel('Energy')

hold off
% save('Graph.mat','yT','ListaCorpos','Corpo');
