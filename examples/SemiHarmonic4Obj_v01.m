%-------------------------------------------------------------------------%
%------------------------------Multibody Dynamic--------------------------%
%------------------------------A toy system-------------------------------% 
%
% Problem:  14 bodies in an harmonic force field
%
% Autor: Carlos Leandro
% Data: 25Fev16
% Version: 
%-------------------------------------------------------------------------%

clc
clear all

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Glabal variables used to 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

addpath('./dynamic')

global World % used to describe the world 
global BodyList % List with body identifiers
global JointList % List with dynamic constrains identifiers
global Bodies % Structure with every rigid bodies in the system
global Joints % Structure with dynamics constrains 
global Simulation
global Motors
global Motor


global TT Err Ke Pe

%%
% World parameters

World.gravity=[0;0;0.0]; % force to be applied on each body
World.ElasticNet = true;  % force are generated using an elastic network
World.Regularistion = true;
World.RFactor = 1e-15;     % regularization factor
World.FMNcontact = false;
World.FMNdensity = false;

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

% Flexible Multibody system
World.Flexible=false;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Multibody system configuration
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

BodyList={'C1','C2','C3','C4','C5','C6','C7','C8','C9','C10'}; % list of system bodies
JointList={'Fix1','Fix2'};      % list of dynamic constrains
Motors={};

World.nbodies=10;
World.njoints=2;
World.nmotors=0;
World.Msize   = World.nbodies*6; % mass matrix size = 6*bodies

% Net Potential
%%
World.potential='SemiHarmonic';

   
%Body C1
%% Geometry
Bodies.C1.geo.m=1; % mass
Bodies.C1.geo.r=1; % radius
Bodies.C1.flexible=false;

% Inertia tensor for body C1
Bodies.C1.geo.JP=diag([1,1,1]);

%List of points in body C1
Bodies.C1.PointsList={};

%List of vectors in body C1
Bodies.C1.VectorsList={};

% Body initial values
Bodies.C1.r=[0,1,0]'; 
Bodies.C1.r_d=[0,0,0]'; 
Bodies.C1.r_dd=[0,0,0]'; 
Bodies.C1.p=[1,0,0,0]'; 
Bodies.C1.p_d=[0,0,0,0]'; 
Bodies.C1.w=[0,0,0]'; 
Bodies.C1.wp=[0,0,0]'; 
Bodies.C1.np=[0,0,0]';
Bodies.C1.exists=true;

Bodies.C1.Nei={'C2'}; % system topology C1<->C2
Bodies.C1.stiffness = [100]; % spring stiffness
Bodies.C1.lengths = [1]; % springs lengths^2
Bodies.C1.num=1;

%Body C2
%%Geometry
Bodies.C2.geo.m=1; % mass
Bodies.C2.geo.r=1; % radius
Bodies.C2.flexible=false;

% Inertia tensor for body C1
Bodies.C2.geo.JP=diag([1,1,1]);

%List of points in body C1
Bodies.C2.PointsList={};

%List of vectors in body C1
Bodies.C2.VectorsList={};

% Body initial values
Bodies.C2.r=[0,1.9,0]'; 
Bodies.C2.r_d=[0,0,0]'; 
Bodies.C2.r_dd=[0,0,0]'; 
Bodies.C2.p=[1,0,0,0]'; 
Bodies.C2.p_d=[0,0,0,0]'; 
Bodies.C2.w=[0,0,0]'; 
Bodies.C2.wp=[0,0,0]'; 
Bodies.C2.np=[0,0,0]';
Bodies.C2.exists=true;

Bodies.C2.Nei={'C1','C3'}; % system topology C1<->C2<->C3
Bodies.C2.stiffness = [100,10];
Bodies.C2.lengths = [1,1]; % springs lengths^2
Bodies.C2.num=2;

%Body C3
%%Geometry
Bodies.C3.geo.m=1; % mass
Bodies.C3.geo.r=1; % radius
Bodies.C3.flexible=false;

% Inertia tensor for body C1
Bodies.C3.geo.JP=diag([1,1,1]);

%List of points in body C1
Bodies.C3.PointsList={};

%List of vectors in body C1
Bodies.C3.VectorsList={};

% Body initial values
Bodies.C3.r=[0,3,0]'; 
Bodies.C3.r_d=[0,0,0]'; 
Bodies.C3.r_dd=[0,0,0]'; 
Bodies.C3.p=[1,0,0,0]'; 
Bodies.C3.p_d=[0,0,0,0]'; 
Bodies.C3.w=[0,0,0]';
Bodies.C3.wp=[0,0,0]'; 
Bodies.C3.np=[0,0,0]';
Bodies.C3.exists=true;

Bodies.C3.Nei={'C2','C4'}; % system topology C2<->C3<->C4
Bodies.C3.stiffness = [10,1];
Bodies.C3.lengths = [1,1]; % springs lengths^2
Bodies.C3.num=2;

%Body C4
%%Geometry
Bodies.C4.geo.m=1; % mass
Bodies.C4.geo.r=1; % radius
Bodies.C4.flexible=false;

% Inertia tensor for body C1
Bodies.C4.geo.JP=diag([1,1,1]);

%List of points in body C1
Bodies.C4.PointsList={};

%List of vectors in body C1
Bodies.C4.VectorsList={};

% Body initial values
Bodies.C4.r=[0,4.1,0]'; 
Bodies.C4.r_d=[0,0,0]'; 
Bodies.C4.r_dd=[0,0,0]'; 
Bodies.C4.p=[1,0,0,0]'; 
Bodies.C4.p_d=[0,0,0,0]'; 
Bodies.C4.w=[0,0,0]'; 
Bodies.C4.wp=[0,0,0]'; 
Bodies.C4.np=[0,0,0]';
Bodies.C4.exists=true;

Bodies.C4.Nei={'C3','C5'}; % system topology C3<->C4<->C5
Bodies.C4.stiffness = [1,.1];
Bodies.C4.lengths = [1,1]; % springs lengths^2
Bodies.C4.num=2;

%Body C5
%%Geometry
Bodies.C5.geo.m=1; % mass
Bodies.C5.geo.r=1; % radius
Bodies.C5.flexible=false;

% Inertia tensor for body C1
Bodies.C5.geo.JP=diag([1,1,1]);

%List of points in body C1
Bodies.C5.PointsList={};

%List of vectors in body C1
Bodies.C5.VectorsList={};

% Body initial values
Bodies.C5.r=[0,5,0]'; 
Bodies.C5.r_d=[0,0,0]'; 
Bodies.C5.r_dd=[0,0,0]'; 
Bodies.C5.p=[1,0,0,0]'; 
Bodies.C5.p_d=[0,0,0,0]'; 
Bodies.C5.w=[0,0,0]'; 
Bodies.C5.wp=[0,0,0]'; 
Bodies.C5.np=[0,0,0]';
Bodies.C5.exists=true;

Bodies.C5.Nei={'C4','C6'}; % system topology C4<->C5<->C6
Bodies.C5.stiffness = [.1,1];
Bodies.C5.lengths = [1,1]; % springs lengths^2
Bodies.C5.num=2;

%Body C6
%% Geometry
Bodies.C6.geo.m=1; % mass
Bodies.C6.geo.r=1; % radius
Bodies.C6.flexible=false;

% Inertia tensor for body C1
Bodies.C6.geo.JP=diag([1,1,1]);

%List of points in body C1
Bodies.C6.PointsList={};

%List of vectors in body C1
Bodies.C6.VectorsList={};

% Body initial values
Bodies.C6.r=[0,5.9,0]'; 
Bodies.C6.r_d=[0,0,0]'; 
Bodies.C6.r_dd=[0,0,0]'; 
Bodies.C6.p=[1,0,0,0]'; 
Bodies.C6.p_d=[0,0,0,0]'; 
Bodies.C6.w=[0,0,0]'; 
Bodies.C6.wp=[0,0,0]'; 
Bodies.C6.np=[0,0,0]';
Bodies.C6.exists=true;

Bodies.C6.Nei={'C5','C7'}; % system topology C1<->C2
Bodies.C6.stiffness = [1,.1]; % spring stiffness
Bodies.C6.lengths = [1,1]; % springs lengths^2
Bodies.C6.num=2;

%Body C7
%%Geometry
Bodies.C7.geo.m=1; % mass
Bodies.C7.geo.r=1; % radius
Bodies.C7.flexible=false;

% Inertia tensor for body C1
Bodies.C7.geo.JP=diag([1,1,1]);

%List of points in body C1
Bodies.C7.PointsList={};

%List of vectors in body C1
Bodies.C7.VectorsList={};

% Body initial values
Bodies.C7.r=[0,7,0]'; 
Bodies.C7.r_d=[0,0,0]'; 
Bodies.C7.r_dd=[0,0,0]'; 
Bodies.C7.p=[1,0,0,0]'; 
Bodies.C7.p_d=[0,0,0,0]'; 
Bodies.C7.w=[0,0,0]'; 
Bodies.C7.wp=[0,0,0]'; 
Bodies.C7.np=[0,0,0]';
Bodies.C7.exists=true;

Bodies.C7.Nei={'C6','C8'}; % system topology C1<->C2<->C3
Bodies.C7.stiffness = [.1,1];
Bodies.C7.lengths = [1,1]; % springs lengths^2
Bodies.C7.num=2;

%Body C8
%%Geometry
Bodies.C8.geo.m=1; % mass
Bodies.C8.geo.r=1; % radius
Bodies.C8.flexible=false;

% Inertia tensor for body C1
Bodies.C8.geo.JP=diag([1,1,1]);

%List of points in body C1
Bodies.C8.PointsList={};

%List of vectors in body C1
Bodies.C8.VectorsList={};

% Body initial values
Bodies.C8.r=[0,8,0]'; 
Bodies.C8.r_d=[0,0,0]'; 
Bodies.C8.r_dd=[0,0,0]'; 
Bodies.C8.p=[1,0,0,0]'; 
Bodies.C8.p_d=[0,0,0,0]'; 
Bodies.C8.w=[0,0,0]';
Bodies.C8.wp=[0,0,0]'; 
Bodies.C8.np=[0,0,0]';
Bodies.C8.exists=true;

Bodies.C8.Nei={'C7','C9'}; % system topology C2<->C3<->C4
Bodies.C8.stiffness = [1,10];
Bodies.C8.lengths = [1,1]; % springs lengths^2
Bodies.C8.num=2;

%Body C9
%%Geometry
Bodies.C9.geo.m=1; % mass
Bodies.C9.geo.r=1; % radius
Bodies.C9.flexible=false;

% Inertia tensor for body C1
Bodies.C9.geo.JP=diag([1,1,1]);

%List of points in body C1
Bodies.C9.PointsList={};

%List of vectors in body C1
Bodies.C9.VectorsList={};

% Body initial values
Bodies.C9.r=[0,9.1,0]'; 
Bodies.C9.r_d=[0,0,0]'; 
Bodies.C9.r_dd=[0,0,0]'; 
Bodies.C9.p=[1,0,0,0]'; 
Bodies.C9.p_d=[0,0,0,0]'; 
Bodies.C9.w=[0,0,0]'; 
Bodies.C9.wp=[0,0,0]'; 
Bodies.C9.np=[0,0,0]';
Bodies.C9.exists=true;

Bodies.C9.Nei={'C8','C10'}; % system topology C3<->C4<->C5
Bodies.C9.stiffness = [10,100];
Bodies.C9.lengths = [1,1]; % springs lengths^2
Bodies.C9.num=2;

%Body C10
%%Geometry
Bodies.C10.geo.m=1; % mass
Bodies.C10.geo.r=1; % radius
Bodies.C10.flexible=false;

% Inertia tensor for body C1
Bodies.C10.geo.JP=diag([1,1,1]);

%List of points in body C1
Bodies.C10.PointsList={};

%List of vectors in body C1
Bodies.C10.VectorsList={};

% Body initial values
Bodies.C10.r=[0,10,0]'; 
Bodies.C10.r_d=[0,0,0]'; 
Bodies.C10.r_dd=[0,0,0]'; 
Bodies.C10.p=[1,0,0,0]'; 
Bodies.C10.p_d=[0,0,0,0]'; 
Bodies.C10.w=[0,0,0]'; 
Bodies.C10.wp=[0,0,0]'; 
Bodies.C10.np=[0,0,0]';
Bodies.C10.exists=true;

Bodies.C10.Nei={'C9'}; % system topology C9<->C10
Bodies.C10.stiffness = [100];
Bodies.C10.lengths = [1]; % springs lengths^2
Bodies.C10.num=1;
%%
% System dynamic constrains

%Ground joint

Joints.Fix1.type='Fix';  % fix body in the space
Joints.Fix1.body_1='C1'; % body identifier

Joints.Fix2.type='Fix';  % fix body in the space
Joints.Fix2.body_1='C10'; % body identifier

%%
% Mass matrix assembly

y_d=[];

nbodies=length(BodyList);
World.M=zeros(nbodies*6);

for indexE=1:nbodies
    BodyName=BodyList{indexE};
    Bodies.(BodyName).g=World.gravity;
    O=zeros(3,3);
    index=(indexE-1)*6+1;
    
    Bodies.(BodyName).index=index; % body index in the mass matrix
    Bodies.(BodyName).forca=[];
    % recursive definition for the initial force
    y_d=[y_d; Bodies.(BodyName).r;Bodies.(BodyName).p;Bodies.(BodyName).r_d;Bodies.(BodyName).w];
end


%% 
% Start simulation

% set integration parameters
t0=0;    % Initial time
t=10.00; % Final time
step=0.001; % Time-step


tspan = [t0:step:t];

% Set integrator and its parameters

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
    plot(yT(:,(index-1)*13+2))
    
    hold on
end
xlabel('z'),ylabel('int')
% print(fig,'Position y z','-dpng')
hold off

%%
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
for indexE=1:nbodies
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
xlabel('iteration'),ylabel('Ke')

%%
save('Graph.mat','World','BodyList','Bodies','Simulation');
