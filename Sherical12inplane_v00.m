%-------------------------------------------------------------------------%
%------------------------------Multibody Dynamic--------------------------%
%------------------------------A toy system-------------------------------% 
% Problem: 12 spherical bodies in a box
% Author: Carlos Leandro
% Data: 2Abr16
% Version: 0.0
%-------------------------------------------------------------------------%
clc
clear all
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Global variables used to 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global World % used to describe the world 
global BodyList % List with body identifiers
global JointList % List with dynamic constrains identifiers
global Bodies % Structure with every rigid bodies in the system
global Joints % Structure with dynamics constrains 


global TT Err

%%
% World parameters

World.gravity=[0;0;-0.00981]; % force to be applied on each body


% Baumgarte Method
World.alpha=5.0;
World.beta=5.0;
World.K=1000;

% Debugging information

World.C1=[];
World.Err=[];

World.delta = 0.01;
World.Error = 1e-7;
World.debuging = false;
World.contact = false;

World.dist=[];

World.FMNcontact = true;
World.FMNdensity = false;

World.box = [0 0 -4 ; 4 4 8];

World.Nx=(World.box(2,1)-World.box(1,1))/World.delta;
World.Ny=(World.box(2,2)-World.box(1,2))/World.delta;
World.Nz=(World.box(2,3)-World.box(1,3))/World.delta;

World.Min=[2^log2(World.Nx)-1 2^log2(World.Ny)-1 2^log2(World.Nz)-1]';
World.Max=[2^(log2(World.Nx)+1)+1 2^(log2(World.Ny)+1)+1 2^(log2(World.Nz)+1)+1]';

World.ecoder_x=zeros(1,fix(World.Max(1)));
World.INTecoder_x=zeros(1,fix(World.Max(1)));
World.ecoder_y=zeros(1,fix(World.Max(2)));
World.INTecoder_y=zeros(1,fix(World.Max(2)));
World.ecoder_z=zeros(1,fix(World.Max(3)));
World.INTecoder_z=zeros(1,fix(World.Max(3)));

% Flexible Multibody system
World.Flexible=false;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Multibody system configuration
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

JointList={'Fix'}; % list of dynamic constrains

%Ground: body Ground
%Geometry

%Body Box
%%Geometry
Bodies.Ground.geo.m=1; % rod mass
Bodies.Ground.geo.h=1; % rod length
Bodies.Ground.geo.r=1; % rod radius
Bodies.Ground.flexible=false;

% Inertia tensor for body C1
Bodies.Ground.geo.JP=diag([1/12*(Bodies.Ground.geo.m*(3*Bodies.Ground.geo.r^2+Bodies.Ground.geo.h^2)),1/2*(Bodies.Ground.geo.m*Bodies.Ground.geo.r^2),...
     1/12*(Bodies.Ground.geo.m*(3*Bodies.Ground.geo.r^2+Bodies.Ground.geo.h^2))]);

%List of points in Ground
Bodies.Ground.PointsList={'BoxC'};
Bodies.Ground.Points.BoxC.sPp=[0,0,0]'; % local frame coordinates

%List of vectors in Ground
Bodies.Ground.VectorsList={};

% Body initial values
Bodies.Ground.r=[0,0,0]'; 
Bodies.Ground.r_d=[0,0,0]'; 
Bodies.Ground.r_dd=[0,0,0]'; 
Bodies.Ground.p=[1,0,0,0]'; 
Bodies.Ground.p_d=[0,0,0,0]'; 
Bodies.Ground.w=[0,0,0]'; 
Bodies.Ground.wp=[0,0,0]'; 
Bodies.Ground.np=[0,0,0]';

Bodies.Ground.contact=true;
Bodies.Ground.box_center ='BoxC';
Bodies.Ground.box = [0 0 0 ; 4 4 0.1];
Bodies.Ground.shape = @(x) abs(x(1))<0 || abs(x(1))>4 || abs(x(2))<0 || abs(x(2))>4 || abs(x(3))<.1;
Bodies.Ground.normal = @(x)[0 0 1]';
Bodies.Ground.ord=1;
Bodies.Ground.exists=true;

% debuging forces
Bodies.Ground.forca=[];

%Body 12 spheres
%%Geometry
init_1=.2;
init_2=.2;
init_3=1.0;

inc=.3;

cont_1=init_1;
cont_2=init_2;
cont_3=init_3;
    
for i=1:2 % 4 .. 138.4674 12 .. 169.5493
    name=sprintf('C%d',i);
    Bodies.(name).geo.m=1; % rod mass
    Bodies.(name).geo.h=1; % rod length
    Bodies.(name).geo.r=1; % rod radius
    
    Bodies.(name).flexible=false;

    % Inertia tensor for spheres
    Bodies.(name).geo.JP=diag([1/12*(Bodies.(name).geo.m*(3*Bodies.(name).geo.r^2+Bodies.(name).geo.h^2)),1/2*(Bodies.(name).geo.m*Bodies.(name).geo.r^2),...
         1/12*(Bodies.(name).geo.m*(3*Bodies.(name).geo.r^2+Bodies.(name).geo.h^2))]);

    %List of points in each sphere
    Bodies.(name).PointsList={'BoxC'};
    Bodies.(name).Points.BoxC.sPp=[0,0,0]'; % local frame coordinates

    %List of vectors in each sphere
    Bodies.(name).VectorsList={};

    % Body initial values
    Bodies.(name).r=[cont_1,cont_2,cont_3]';
    
    if cont_1>3*init_1
        cont_1=init_1;
        cont_3=cont_3+inc;
    end
    
    if cont_3>3*init_3
        cont_1=init_1;
        cont_3=init_3;
        cont_1=cont_1+inc;
    end
    
    cont_3=cont_3+inc;
    
    
    Bodies.(name).r_d=[0,0,0]'; 
    Bodies.(name).r_dd=[0,0,0]'; 
    Bodies.(name).p=[1,0,0,0]'; 
    Bodies.(name).p_d=[0,0,0,0]'; 
    Bodies.(name).w=[0,0,0]'; 
    Bodies.(name).wp=[0,0,0]'; 
    Bodies.(name).np=[0,0,0]';
    
    Bodies.(name).contact=true;
    Bodies.(name).box_center ='BoxC';
    Bodies.(name).box = [-.2 -.2 -.2 ; .2 .2 .2];
    Bodies.(name).shape = @(x)x'*x< 0.01 ;
    Bodies.(name).normal = @(x)x/norm(x);
    Bodies.(name).ord=i+1;
    Bodies.(name).exists=true;
    
    % debuging forces
    Bodies.(name).forca=[];
end
%%
% System dynamic constrains

%Ground joint

Joints.Fix.type='Fix';  % fix body in the space
Joints.Fix.body_1='Ground'; % body identifier

%Sherical joint linking body Ground and body C1
Joints.J1.type='She';  % spherical joint between body_1 and body_2
Joints.J1.body_1='Ground'; % body_1 identifier
Joints.J1.body_2='Box'; % body_2 identifier
Joints.J1.point='P1';  % point identifier

% Transport Multibody system information

BodyList=fieldnames(Bodies);
%st={'BodyLiC1','Ground'};

World.nbodies = length(BodyList);  % number of bodies in the system 
World.njoints = length(JointList); % number of joints in the system
World.NNodes  = 0;                 % count number of nodes in the system
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
    
    % recursive definition for the initial force
    y_d=[y_d; Bodies.(BodyName).r;Bodies.(BodyName).p;Bodies.(BodyName).r_d;Bodies.(BodyName).w];
    
end


%% 
% Start simulation

% set integration parameters
t0=0;    % Initial time
t=7.0; % Final time 7.22
step=0.01; % Time-step


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
    plot(yT(:,(index-1)*13+2), yT(:,(index-1)*13+3))
    
    hold on
end
xlabel('y'),ylabel('z')
% print(fig,'Positionyz','-dpng')

hold off

fig=figure;
for index=1:nbodies
    BodyName=BodyList{index};
    plot(yT(:,(index-1)*13+1), yT(:,(index-1)*13+3))   
    hold on
end
xlabel('x'),ylabel('z')
% print(fig,'Positionxz','-dpng')
hold off
% Velocidades**2
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

% Error on the system constrains
fig=figure;
plot( Err) 
xlabel('iteration'),ylabel('Error')

hold off
% z vs time
fig=figure;
for indexE=1:nbodies
    L=yT(:,(indexE-1)*13+3);
    plot(T, L) 
    hold on
end
xlabel('T'),ylabel('z')

% Used to movie criation
save('Graph.mat','yT','BodyList','Bodies');
