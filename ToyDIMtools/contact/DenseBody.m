clc
clear all

global World % used to describe the world 
global Bodies 


% elera
% 1                                    22.480600 seconds
% 2                                    44.374845 -- 47.549657
% 3                                    72.708356 --box 78.527885 -- shape 74.170821                                  

World.delta = 0.1;
World.Error = 1e-7;

World.box = [-10 -10 -10 ; 10 10 10];

World.Nx=(World.box(2,1)-World.box(1,1))/World.delta;
World.Ny=(World.box(2,2)-World.box(1,2))/World.delta;
World.Nz=(World.box(2,3)-World.box(1,3))/World.delta;

World.Min=[2^log2(World.Nx)-1 2^log2(World.Ny)-1 2^log2(World.Nz)-1]';
World.Max=[2^(log2(World.Nx)+1) 2^(log2(World.Ny)+1) 2^(log2(World.Nz)+1)]';

World.ecoder_x=zeros(1,fix(World.Max(1)));
World.INTecoder_x=zeros(1,fix(World.Max(1)));
World.ecoder_y=zeros(1,fix(World.Max(2)));
World.INTecoder_y=zeros(1,fix(World.Max(2)));
World.ecoder_z=zeros(1,fix(World.Max(3)));
World.INTecoder_z=zeros(1,fix(World.Max(3)));

Bodies.C1.box = [-1 -1 -1 ; 1 1 1];
Bodies.C1.r = [0 0 .3]'; %[0.05 20 20]'; % mass center
Bodies.C1.shape = @(x)abs(x(1))<1 && abs(x(2))<1 && abs(x(3))<1;
Bodies.C1.A = eye(3);
Bodies.C1.ord=1;


Bodies.C2.box = [-1 -1 -1 ; 1 1 1];
Bodies.C2.r = [0 0 .5]'; % mass center
Bodies.C2.shape = @(x)abs(x(1))<1 && abs(x(2))<1 && abs(x(3))<1;
Bodies.C2.A = eye(3);
Bodies.C2.ord=2;

Bodies.C3.shape = sphere;
Bodies.C3.box = [-1 -1 -1 ; 1 1 1];
Bodies.C3.r = [3 5 5]'; % mass center
Bodies.C3.A = eye(3);
Bodies.C3.ord=3;

World.BodyList={'C2','C1'};%,'C3'};

World.nbodies=length(World.BodyList);

tic
for indexE=1:World.nbodies
    BodyName=World.BodyList{indexE};
    
    Pic(Bodies.(BodyName),BodyName,indexE);
end
toc