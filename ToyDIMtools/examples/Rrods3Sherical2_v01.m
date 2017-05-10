%-------------------------------------------------------------------------%
%------------------------------Multibody Dynamic--------------------------%
%------------------------------A toy system-------------------------------% 
% Problem: 3 rods linked using two spherical joints 3D
% Author: Carlos Leandro
% Data: 25Fev16
% Version: 1.0
%-------------------------------------------------------------------------%
clc
clear all
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Global variables used to 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
addpath('../dynamic')

global World % used to describe the world 
global BodyList % List with body identifiers
global JointList % List with dynamic constrains identifiers
global Bodies % Structure with every rigid bodies in the system
global Joints % Structure with dynamics constrains 
global Simulation
global Motors
global Motor

global TT Err

%%
% World parameters

World.gravity=[0;0;-0.00981]; % force to be applied on each body
World.ElasticNet = false;  % force are generated using and elastic network
World.Regularistion = true;
World.RFactor = 1e-15;     % regularization factor
World.FMNcontact = false;
World.FMNdensity = false;

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

Bodies.Ground.geo.m=1; % massa
Bodies.Ground.flexible=false;


Bodies.Ground.geo.JP=diag([1,1,1]); % Moment of inertia

% Points in body Ground
Bodies.Ground.PointsList={'P1'};
Bodies.Ground.Points.P1.sPp=[0,0.5,0]'; % local frame coordinates

% Vectors in body Ground
Bodies.Ground.VectorsList={};

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

%List of vectors in body C2
Bodies.C2.VectorsList={};

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

%%
% System dynamic constrains

%Ground joint

Joints.Fix.type='Fix';  % fix body in the space
Joints.Fix.body_1='Ground'; % body identifier

%Sherical joint linking body Ground and body C1
Joints.J1.type='She';  % spherical joint between body_1 and body_2
Joints.J1.body_1='Ground'; % body_1 identifier
Joints.J1.body_2='C1'; % body_2 identifier
Joints.J1.point='P1';  % point identifier

% Sherical joint linking body C1 and body C2
Joints.J2.type='She';  % revolute joint between body_1 and body_2
Joints.J2.body_1='C1'; % body_1 identifier
Joints.J2.body_2='C2'; % body_2 identifier
Joints.J2.point='P2';  % point identifier

% Sherical joint linking body C2 and body C3
Joints.J3.type='She';  % revolute joint between body_1 and body_2
Joints.J3.body_1='C2'; % body_1 identifier
Joints.J3.body_2='C3'; % body_2 identifier
Joints.J3.point='P3';  % point identifier

% Transport Multibody system information

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
t=100.00; % Final time
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

%%
% Output: systems c-of-m yz-projection

fig=figure;
for index=1:nbodies 
    BodyName=BodyList{index};
    plot(yT(:,(index-1)*13+2), yT(:,(index-1)*13+3))
    
    hold on
end
xlabel('y'),ylabel('z')
% print(fig,'Positionyz','-dpng')

hold off

%%
% Output: systems c-of-m xz-projection
fig=figure;
for index=1:nbodies
    BodyName=BodyList{index};
    plot(yT(:,(index-1)*13+1), yT(:,(index-1)*13+3))   
    hold on
end
xlabel('x'),ylabel('z')
% print(fig,'Positionxz','-dpng')
hold off

%%
% Output: Velocidades**2

fig=figure;
for indexE=1:nbodies
    L=yT(:,(indexE-1)*13+8:(indexE-1)*13+10);
    v=[];
    for i=1:length(L)
        v=[v L(i,:)*L(i,:)'];
    end
    plot(T, v) 
    hold on
end
xlabel('T'),ylabel('v^2[m/s]')
%print(fig,'velocity','-dpng')
hold off

%%
% Output: Error on the system constrains

fig=figure;
plot( Err) 
xlabel('iteration'),ylabel('Error')

