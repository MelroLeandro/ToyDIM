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

global World % used to describe the world 
global BodyList % List with body identifiers
global JointList % List with dynamic constrains identifiers
global Bodies % Structure with every rigid bodies in the system
global Joints % Structure with dynamis constrains 


global TT Err Ke Pe

%%
% World parameters

World.g=[0;0;0.01];       % force to be applied on each body
World.ElasticNet = true;  % force are generated using and elastic network

% Contact
World.n=1.5;
World.e=0.6;
World.k=2*10^4;
World.mu=0.74;

% Baumgarte Method
World.alpha=5.0;
World.beta=5.0;

% System stiffnesses
% Liner spring properties
World.stiffness=1;

% Debugging information

World.C1=[];
World.Err=[];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Multibody system configuration:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%
% Geometry

for i=0:3
    for j=0:3
        
        bodyName=sprintf('C%dx%d',i,j);       
        
        % system masses and geometry
        Bodies.(bodyName).geo.m=1;      % node mass
        Bodies.(bodyName).geo.r=1;      % node radius
        Bodies.(bodyName).geo.charge=0; % node chage

        % Inertia tensor for body 
        Bodies.(bodyName).geo.JP=eye(3);

        %List of points a body 
        Bodies.(bodyName).PointsList={'P1'};

        Bodies.(bodyName).Points.P1.sPp=[0,0,0]';

        %List of vectors a body 
        Bodies.(bodyName).VectorsList={'V1'};

        Bodies.(bodyName).Vectors.V1.sP=[1,0,0]';

        % Body initial values
        Bodies.(bodyName).coupled=true; % body is couled to the elastic network
        Bodies.(bodyName).r0=[0,i,j]'; % equilibrium positions
        Bodies.(bodyName).r=[0,i,j]';  % initial positions
        Bodies.(bodyName).r_d=[0,0,0]'; 
        Bodies.(bodyName).r_dd=[0,0,0]'; 
        Bodies.(bodyName).p0=[1,0,0,0]'; % equilibrium orientation
        Bodies.(bodyName).p=[1,0,0,0]';  % initial orientation
        Bodies.(bodyName).p_d=[0,0,0,0]'; 
        Bodies.(bodyName).w=[0,0,0]'; 
        Bodies.(bodyName).wp=[0,0,0]'; 
        Bodies.(bodyName).np=[0,0,0]';
    end
end

%%
% System topology

Bodies.C0x0.Nei={'C0x1','C1x0'};
Bodies.C0x0.num=2;
Bodies.C0x1.Nei={'C0x0','C0x2','C1x1'};
Bodies.C0x1.num=3;
Bodies.C0x2.Nei={'C0x2','C0x3','C1x2'};
Bodies.C0x2.num=3;
Bodies.C0x3.Nei={'C0x2','C1x3'};
Bodies.C0x3.num=2;
Bodies.C1x0.Nei={'C0x0','C1x1','C2x0'};
Bodies.C1x0.num=3
Bodies.C1x1.Nei={'C0x1','C1x0','C1x2','C2x1'};
Bodies.C1x1.num=4;
Bodies.C1x2.Nei={'C0x2','C1x2','C1x3','C2x2'};
Bodies.C1x2.num=4;
Bodies.C1x3.Nei={'C1x3','C2x2','C3x3'};
Bodies.C1x3.num=3;
Bodies.C2x0.Nei={'C1x0','C2x1','C3x0'};
Bodies.C2x0.num=3;
Bodies.C2x1.Nei={'C1x1','C2x0','C2x2','C3x1'};
Bodies.C2x1.num=4;
Bodies.C2x2.Nei={'C1x2','C2x2','C2x3','C3x2'};
Bodies.C2x2.num=4;
Bodies.C2x3.Nei={'C1x3','C2x2','C3x3'};
Bodies.C2x3.num=3;
Bodies.C3x0.Nei={'C2x0','C3x1'};
Bodies.C3x0.num=2;
Bodies.C3x1.Nei={'C2x1','C3x0','C3x2'};
Bodies.C3x1.num=3;
Bodies.C3x2.Nei={'C2x2','C3x2','C3x3'};
Bodies.C3x2.num=3;
Bodies.C3x3.Nei={'C2x3','C3x2'};
Bodies.C3x3.num=2;
%%
% Contact matrix

BodyList=fieldnames(Bodies); % list of system bodies

NumBodies= length(BodyList)


%%
% System center of mass and the moment of inertia tensor

v=[0,0,0]';
m=0;

for index=1:length(BodyList)
    
    BodyName=BodyList{index};
    bm = Bodies.(BodyName).geo.m;
    x0 = Bodies.(BodyName).r0;
    
    v = v + bm*x0;  
    m=m+bm;
end
World.center_mass=v/m; % System center of mass

%%
% We assume here that World.center_mass=(0,0,0)
I=eye(3);
JP=zeros(3);

for index=1:length(BodyList)
    
    BodyName=BodyList{index};
    x0 = (Bodies.(BodyName).r0-World.center_mass);
    Bodies.(BodyName).r0=x0;
    Bodies.(BodyName).r = (Bodies.(BodyName).r-World.center_mass);
    
    JP = JP + bm*(x0'*x0*I-x0*x0'); 
end

World.JP=JP;                        % moment of inertia tensor
[World.V,World.JP] = eig(World.JP); % V'*Jp*V is diagonal

for index=1:length(BodyList)
    
    BodyName=BodyList{index};
    Bodies.(BodyName).r0 = World.V'*Bodies.(BodyName).r0;
    Bodies.(BodyName).r = World.V'*Bodies.(BodyName).r;
end

% Reset system center of mass
World.center_mass=[0,0,0]';
%%
% 

%%
%Ground: frame fixed in space at the center of mass of all 
%        the representative atoms and oriented along the 
%         principal moments of inertia

Bodies.Ground.geo.m=1; % massa

% Inertia tensor for body Ground
Bodies.Ground.geo.JP=World.JP;

% Points in body C1
Bodies.Ground.PointsList={'P1'};
Bodies.Ground.Points.P1.sPp=[0,0.5,0]'; % local frame coordenates

% Vectors in body C1
Bodies.Ground.VectorsList={'V1'};
Bodies.Ground.Vectors.V1.sP=[1,0,0]'; % local frame coordenates

% Body initial values
Bodies.Ground.coupled=false; % body is not couled to the elastic network
Bodies.Ground.r=[0,0,0]';  % Body initial position in global coordenates
Bodies.Ground.r_d=[0,0,0]';  % initial velocity
Bodies.Ground.r_dd=[0,0,0]'; % initial acceleration
Bodies.Ground.p=[1,0,0,0]';  % initial euler parameters
Bodies.Ground.p_d=[0,0,0,0]';% derivada dos parametros de Euler
Bodies.Ground.w=[0,0,0]';    % initial angular velocity
Bodies.Ground.wp=[0,0,0]';   % initial angular aceleration
Bodies.Ground.np=[0,0,0]';   % initial momento

%%
% Update list of bodies
BodyList=fieldnames(Bodies); % list of system bodies

JointList={'Fix'}; % list of dynamic constrains

%%
% System dynamic constrains

%Ground joint
Joints.Fix.type='Fix';  % fix body in the space
Joints.Fix.body_1='Ground'; % body identidier

%%
% The global mass matrix 




%%
% The global stiffness matrix


y_d=[];

nbodies=length(BodyList);
world.M=zeros(nbodies*6);
for indexE=1:nbodies
    BodyName=BodyList{indexE};
    Bodies.(BodyName).g=World.g;
    O=zeros(3,3);
    index=(indexE-1)*6+1;
    
    Bodies.(BodyName).index=index; % body index in the mass matrix
    
    % recursive definition for the initial force
    y_d=[y_d; Bodies.(BodyName).r;Bodies.(BodyName).p;Bodies.(BodyName).r_d;Bodies.(BodyName).w];
end


%% 
% Start simulation

% set integration parametters
t0=0;    % Initial time
t=10.00; % Final time
step=0.001; % Time-step


tspan = [t0:step:t];

% Set integrator and its parametters

fprintf('\n\n ODE45\n\n')
tol=1e-5;
options=odeset('RelTol',tol,'Stats','on','OutputFcn',@odeOUT); 

tic
[T, yT]= ode45(@updateaccel, tspan, y_d, options);
timeode45=toc

%%
% Graphic output

fig=figure;
for index=1:nbodies 
    BodyName=BodyList{index};
    plot(yT(:,(index-1)*13+2), yT(:,(index-1)*13+3))
    
    hold on
end
xlabel('y'),ylabel('z')
legend('position y','position z')
% print(fig,'Position y z','-dpng')
hold off

fig=figure;
for index=1:nbodies
    BodyName=BodyList{index};
    plot(yT(:,(index-1)*13+1), yT(:,(index-1)*13+3))   
    hold on
end
xlabel('x'),ylabel('z')
legend('position x','position z')
% print(fig,'Position x z','-dpng')
hold off

fig=figure;
for index=1:nbodies
    BodyName=BodyList{index};
    plot(yT(:,(index-1)*13+1), yT(:,(index-1)*13+2))   
    hold on
end
xlabel('x'),ylabel('z')
legend('position x','position y')

% print(fig,'Position x z','-dpng')
hold off
% Velocidades
fig=figure;
for indexE=1:nbodies %ciclo for actua sobre todos os diferentes esferas
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
