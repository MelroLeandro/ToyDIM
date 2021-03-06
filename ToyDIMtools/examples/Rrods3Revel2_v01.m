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

%% ["v11","v12","v13","v21","v22","v23","r11","r12","r13","r21","r22","r23","r31","r32","r33","q11","q12","q13","q21","q22","q23","q31","q32","q33"];
World.V=[]
%%
% World parameters

World.gravity=[0;0;-0.00981]; % force to be applied on each body
World.ElasticNet = false;  % force are generated using and elastic network
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

BodyList={'Ground','C1','C2','C3'}; % list of system bodies
JointList={'Fix','J1','J2','J3'}; % list of dynamic constrains
Motors={};

%Ground: body C1
%Geometry

Bodies.Ground.geo.m=1; % mass
Bodies.Ground.flexible=false;

% Inertia tensor for body Ground
Bodies.Ground.geo.JP=diag([1,1,1]);

% Points in body C1
Bodies.Ground.PointsList={'P1'};
Bodies.Ground.Points.P1.sPp=[0,0.5,0]'; % local frame coordinates

% Vectors in body C1
Bodies.Ground.VectorsList={'V1'};
Bodies.Ground.Vectors.V1.sP=[1,0,0]'; % local frame coordinates

% Body initial values
Bodies.Ground.r=[0,0.0,0]'; % Body initial position in global coordinates
Bodies.Ground.r_d=[0,0,0]';  % initial velocity
Bodies.Ground.r_dd=[0,0,0]';  % initial acceleration
Bodies.Ground.p=[1,0,0,0]';  % initial Euler parameters
Bodies.Ground.p_d=[0,0,0,0]';% Euler parameters derivative
Bodies.Ground.w=[0,0,0]';    % initial angular velocity
Bodies.Ground.wp=[0,0,0]';  % initial angular acceleration
Bodies.Ground.np=[0,0,0]';   % initial moment
Bodies.Ground.exists=true;

%Body C1 a Rod
%%Geometry
Bodies.C1.geo.m=1; % rod mass
Bodies.C1.geo.h=1; % rod length
Bodies.C1.geo.r=1; % rod radius
Bodies.C1.flexible=false;

% Inertia tensor for body C1
Bodies.C1.geo.JP=diag([1/12*(Bodies.C1.geo.m*(3*Bodies.C1.geo.r^2+Bodies.C1.geo.h^2)),1/2*(Bodies.C1.geo.m*Bodies.C1.geo.r^2),...
     1/12*(Bodies.C1.geo.m*(3*Bodies.C1.geo.r^2+Bodies.C1.geo.h^2))]);

%List of points in body C1
Bodies.C1.PointsList={'P1','P2'};

Bodies.C1.Points.P1.sPp=[0,-0.5,0]';
Bodies.C1.Points.P2.sPp=[0,0.5,0]';

%List of vectors in body C1
Bodies.C1.VectorsList={'V1','V2'};

Bodies.C1.Vectors.V1.sP=[1,0,0]';
Bodies.C1.Vectors.V2.sP=[1,0,0]';

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


%Body C2 a Rod
%%Geometry
Bodies.C2.geo.m=1; % rod mass
Bodies.C2.geo.h=1; % rod length
Bodies.C2.geo.r=1; % rod radius
Bodies.C2.flexible=false;

% Inertia tensor for body C2
Bodies.C2.geo.JP=diag([1/12*(Bodies.C2.geo.m*(3*Bodies.C2.geo.r^2+Bodies.C2.geo.h^2)),1/2*(Bodies.C2.geo.m*Bodies.C2.geo.r^2),...
     1/12*(Bodies.C2.geo.m*(3*Bodies.C2.geo.r^2+Bodies.C2.geo.h^2))]);

%List of points in body C2
Bodies.C2.PointsList={'P2','P3'};

Bodies.C2.Points.P2.sPp=[0,-0.5,0]';
Bodies.C2.Points.P3.sPp=[0,0.5,0]';

%List of vectors in body C3
Bodies.C2.VectorsList={'V2','V3'};

Bodies.C2.Vectors.V2.sP=[1,0,0]';
Bodies.C2.Vectors.V3.sP=[1,0,0]';

% Body initial values
Bodies.C2.r=[0,2,0]'; 
Bodies.C2.r_d=[0,0,0]'; 
Bodies.C2.r_dd=[0,0,0]'; 
Bodies.C2.p=[1,0,0,0]'; 
Bodies.C2.p_d=[0,0,0,0]'; 
Bodies.C2.w=[0,0,0]'; 
Bodies.C2.wp=[0,0,0]'; 
Bodies.C2.np=[0,0,0]';
Bodies.C2.exists=true;

%Body C3 a Rod
%%Geometry
Bodies.C3.geo.m=1; % rod mass
Bodies.C3.geo.h=1; % rod length
Bodies.C3.geo.r=1; % rod radius
Bodies.C3.flexible=false;

% Inertia tensor for body C3
Bodies.C3.geo.JP=diag([1/12*(Bodies.C3.geo.m*(3*Bodies.C3.geo.r^2+Bodies.C3.geo.h^2)),1/2*(Bodies.C3.geo.m*Bodies.C3.geo.r^2),...
     1/12*(Bodies.C3.geo.m*(3*Bodies.C3.geo.r^2+Bodies.C3.geo.h^2))]);

%List of points in body C3
Bodies.C3.PointsList={'P3','P4'};

Bodies.C3.Points.P3.sPp=[0,-0.5,0]';
Bodies.C3.Points.P4.sPp=[0,0.5,0]';
%List of vectors in body C3
Bodies.C3.VectorsList={'V3'};

Bodies.C3.Vectors.V3.sP=[1,0,0]';

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

%%
% System dynamic constrains

%Ground joint

Joints.Fix.type='Fix';  % fix body in the space
Joints.Fix.body_1='Ground'; % body identifier

%Sherical joint linking body C1 and body C2
Joints.J1.type='Rev';  % spherical joint between body_1 and body_2
Joints.J1.body_1='Ground'; % body_1 identifier
Joints.J1.body_2='C1'; % body_2 identifier
Joints.J1.point='P1';  % point identifier
Joints.J1.vector='V1'; % vector identifier

% Revolute joint linking body C2 and body C3
Joints.J2.type='Rev';  % revolute joint between body_1 and body_2
Joints.J2.body_1='C1'; % body_1 identifier
Joints.J2.body_2='C2'; % body_2 identifier
Joints.J2.point='P2';  % point identifier
Joints.J2.vector='V2'; % vector identifier

% Revolute joint linking body C2 and body C3
Joints.J3.type='Rev';  % revolute joint between body_1 and body_2
Joints.J3.body_1='C2'; % body_1 identifier
Joints.J3.body_2='C3'; % body_2 identifier
Joints.J3.point='P3';  % point identifier
Joints.J3.vector='V3'; % vector identifier

% Tansport Multibody system information

World.nbodies = length(BodyList);  % number of bodies in the system 
World.njoints = length(JointList); % number of joints in the system
World.nmotors = length(Motors); % number of joints in the system
World.NNodes  = 0 ;            % count number of nodes in the system
World.Msize   = World.NNodes*6+World.nbodies*6; % mass matrix size

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
t=300.00; % Final time
step=0.001; % Time-step


tspan = [t0:step:t];

% Set integrator and its parameters

fprintf('\n\n ODE45\n\n')
tol=1e-5;
options=odeset('RelTol',tol,'Stats','on','OutputFcn',@odeOUT); 

tic
[T, yT]= ode45(@updateaccel, tspan, y_d, options);
timeode45=toc


csvwrite('dataSetrev.csv',World.V)

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

% save('Graph.mat','yT','ListaCorpos','Corpo');
