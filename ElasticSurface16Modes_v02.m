%-------------------------------------------------------------------------%
%------------------------------Multibody Dynamic--------------------------%
%------------------------------A toy system-------------------------------% 
% Problem:  elastic rod defined using 3 nodes
% Author: Carlos Leandro
% Data: 11Mar16
% Version: 
%-------------------------------------------------------------------------%
clc
clear all
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Global variables used to 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global World    % used to describe the world 
global BodyList % List with body identifiers
global JointList % List with dynamic constrains identifiers
global Bodies   % Structure with every rigid bodies in the system
global Joints   % Structure with dynamics constrains 


global TT Err Ke Pe


global Simulation % simulation data

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

% Flexible Multibody system
World.Flexible=true;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Multibody system configuration:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

BodyList={'C1'}; % list of system bodies
JointList={'Fix'}; % list of dynamic constrains
%JointList={};

NumBodies= length(BodyList);
%%
% Geometry: 'C1' is a flexible body

bodyName='C1';
Bodies.(bodyName).flexible=true;

% Body fixed frames: implicitly included by removing the nodal coordinates
Bodies.(bodyName).FixedFrameNodeOrder=[1]; % nodes order in finite element discrition
Bodies.(bodyName).NumFixedFrameNodes=1;    % number of fixed nodes

count=1; 
countd=1;
for i=1:4
    for j=1:4
        %
        % System topology

        node=sprintf('C%dx%d',i,j);  
      
        for ii=1:Bodies.(bodyName).NumFixedFrameNodes
            if count == Bodies.(bodyName).FixedFrameNodeOrder(ii)
                Bodies.(bodyName).node.(node).idx = 0;          % nodes indexation
            else
                Bodies.(bodyName).node.(node).idx = countd;      % nodes indexation
                countd=countd+1;                                  % nodes count
            end
        end
        count=count+1;
        
        % system masses and geometry
        Bodies.(bodyName).node.(node).m=0.001;      % node mass
        Bodies.(bodyName).node.(node).mu=0.001;      % local rot inertia
        Bodies.(bodyName).node.(node).charge=0; % node change
        Bodies.(bodyName).node.(node).x=[0,i*.5,j*.5]';  % initial global positions
        Bodies.(bodyName).node.(node).deltaP=[0.0,0.0,0.01*j]';
%         if i==16
%             Bodies.(bodyName).node.(node).deltaP=[0,0.1,0.01]';  % relative displacement
%         else
%             Bodies.(bodyName).node.(node).deltaP=[0,0,0]';
%         end
        Bodies.(bodyName).node.(node).w=[0,0,0]';
        Bodies.(bodyName).node.(node).deltaP_d=[0,0,0]';
        Bodies.(bodyName).node.(node).omegaP=[0,0,0]';  % nodal angular relative velocity
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
            Bodies.(bodyName).node.(NodeName).f = Bodies.(bodyName).V'*Bodies.(bodyName).node.(NodeName).f;
       end
    end % is a flexible body
end

%%
%  C1  : frame fixed in space at the center of mass of all 
%        the representative atoms and oriented along the 
%        principal moments of inertia
bodyName = 'C1';
Bodies.(bodyName).GlobalNodeList   = fieldnames(Bodies.(bodyName).node);
Bodies.(bodyName).GlobalNumberNodes = length(Bodies.(bodyName).GlobalNodeList);

listnodes=[];
for idx=1:Bodies.(bodyName).GlobalNumberNodes   
    NodeName = Bodies.(bodyName).GlobalNodeList{idx};
    if  Bodies.(bodyName).node.(NodeName).idx > 0
        listnodes=[listnodes ; Bodies.(bodyName).GlobalNodeList{idx}];
    end
end 

Bodies.C1.NodeList   = listnodes; % List of flexible nodes in the body
[Bodies.C1.NumberNodes,~] = size(listnodes); % Number of flexible nodes in the body

% Points in body C1
Bodies.C1.PointsList    = {'P1'};
Bodies.C1.Points.P1.sPp = [0,0,0]'; % local frame coordinates

% Vectors in body C1
Bodies.C1.VectorsList   = {'V1'};
Bodies.C1.Vectors.V1.sP = [1,0,0]'; % local frame coordinates

% Body initial values
Bodies.C1.r   = Bodies.C1.center_mass;
Bodies.C1.r_d = [0,0,0]';    % initial velocity
Bodies.C1.r_dd= [0,0,0]';    % initial acceleration
Bodies.C1.p   = [1,0,0,0]';  % initial Euler parameters
Bodies.C1.p_d = [0,0,0,0]';  % Euler parameters derivative
Bodies.C1.w   = [0,0,0]';    % initial angular velocity
Bodies.C1.wp  = [0,0,0]';    % initial angular acceleration
Bodies.C1.np  = [0,0,0]';    % initial moment

% Material properties
Bodies.C1.stiffness = 10000;


%%
% System dynamic constrains

%Ground joint
Joints.Fix.type   = 'Fix';  % fix body in the space
Joints.Fix.body_1 = 'C1';   % body identifier

% Multibody system information
World.nbodies = length(BodyList);  % number of bodies in the system 
World.njoints = length(JointList); % number of joints in the system

NNodes = 0; % count the total number of nodes in the all system

for  indexE=1:World.nbodies
    BodyName=BodyList{indexE};
    NNodes = NNodes + Bodies.(BodyName).NumberNodes;
end

World.NNodes  = NNodes;            % count number of nodes in the system
World.Msize   = World.NNodes*6+World.nbodies*6; % mass matrix size = 6*bodies + 6*nodes

%%
% The global mass matrix 

y_d=[];

World.M=zeros(World.Msize); % Flexible body mass matrix allocation

count = 1;
for indexE = 1:World.nbodies
    BodyName=BodyList{indexE};
    
    Bodies.(BodyName).g = World.g; % force to be applied in each node
    
    O = zeros(3,3);
    
    index = count;
    count = count+6;
    
    Bodies.(BodyName).index = index; % body index in the mass matrix
    
    % recursive definition for the initial force
    y_d=[y_d; Bodies.(BodyName).r;Bodies.(BodyName).p;Bodies.(BodyName).r_d;Bodies.(BodyName).w];
    if Bodies.(BodyName).flexible % have a flexible part
        
        for  index1 = 1:Bodies.(bodyName).NumberNodes
           NodeName = Bodies.(bodyName).NodeList(index1,:);
           
           if Bodies.(bodyName).node.(NodeName).idx>0
               deltaP   = Bodies.(bodyName).node.(NodeName).deltaP;   % equilibrium positions
               omegaP   = Bodies.(bodyName).node.(NodeName).omegaP;   % equilibrium orientation
               deltaP_d = Bodies.(bodyName).node.(NodeName).deltaP_d; % 
               omegaP_d = Bodies.(bodyName).node.(NodeName).omegaP_d; % 

               y_d=[y_d; deltaP; omegaP; deltaP_d; omegaP_d];

               index = count;
               count = count+6;
               Bodies.(bodyName).node.(NodeName).y_index=index;
           end
        end
    end % have a flexible part
end

%%
% Contact matrix K; The global stiffness matrix G;


for idx=1:length(BodyList)
    bodyName=BodyList{idx};  
    
    if Bodies.(bodyName).flexible
        
        Bodies.(bodyName).K = zeros(Bodies.(bodyName).GlobalNumberNodes);
        Bodies.(bodyName).G = zeros(6*Bodies.(bodyName).NumberNodes+6);

        NNodes = NNodes + Bodies.(bodyName).NumberNodes;

        for  index1=1:Bodies.(bodyName).NumberNodes

            NodeName = Bodies.(bodyName).NodeList(index1,:);

            Gii = zeros(3);
            Wii = zeros(3);

            if Bodies.(bodyName).node.(NodeName).idx > 0
                
                i   = Bodies.(bodyName).node.(NodeName).y_index; 
                delta_i = Bodies.(bodyName).node.(NodeName).xP; % equilibrium positions
                omega_i = Bodies.(bodyName).node.(NodeName).w; % equilibrium orientation

                for idx2=1:Bodies.(bodyName).node.(NodeName).num
                    NodeName2=Bodies.(bodyName).node.(NodeName).Nei{idx2};
                    delta_j = Bodies.(bodyName).node.(NodeName2).xP; % equilibrium positions
                    omega_j = Bodies.(bodyName).node.(NodeName2).w; % equilibrium orientation

                    dr = delta_i-delta_j;
                    dp = omega_i-omega_j;

                    Gij = Bodies.(bodyName).stiffness*(dr*dr')/(dr'*dr);
                    Wij = zeros(3);

                    Gii = Gii + Gij;
                    Wii = Wii + Wij;

                    if Bodies.(bodyName).node.(NodeName2).idx>0
                        j   = Bodies.(bodyName).node.(NodeName2).y_index;
                        Bodies.(bodyName).G(i:i+5,j:j+5)=[-Gij zeros(3,3); zeros(3,3) -Wij];
                    end
                end
                   
                Bodies.(bodyName).G(i:i+5,i:i+5)=[Gii zeros(3,3); zeros(3,3) Wii];
            end
        end
    end % body is flexible
end

%% 
% Start simulation
        for indexE = 1:World.nbodies
            BodyName=BodyList{indexE};

            for indexP=1:length(Bodies.(BodyName).PointsList)
                pointname=Bodies.(BodyName).PointsList{indexP};

                Simulation.(BodyName).(pointname)= [];
             end

            if Bodies.(BodyName).flexible % have a flexible part

                for  index1 = 1:Bodies.(BodyName).NumberNodes
                   NodeName = Bodies.(BodyName).NodeList(index1,:);
                   Simulation.(BodyName).(NodeName)= [];
                end
            end
        end

% set integration parameters
t0   = 0;    % Initial time
t    = 300.00; % Final time
step = 0.001; % Time-step


tspan = [t0:step:t];

% Set integrator and its parameters

fprintf('\n\n ODE45\n\n')
tol = 1e-5;
options = odeset('RelTol',tol,'Stats','on','OutputFcn',@odeOUT); 

tic
[T, yT] = ode45(@updateaccel, tspan, y_d, options);
timeode45 = toc

%%
% Graphic output: Y-Z

fig = figure;
for index = 1:World.nbodies
    BodyName = BodyList{index};
    %y = yT(:,(index-1)*13+1:(index-1)*13+3)';
    %plot(y(2,:), y(3,:))
  
    hold on

%     for indexP=1:length(Bodies.(BodyName).PointsList)
%         pointname=Bodies.(BodyName).PointsList{indexP};
% 
%         z=Simulation.(BodyName).(pointname);
%         plot(z(2,:), z(3,:), 'r')
%     end

    if Bodies.(BodyName).flexible % have a flexible part

        for  index1 = 1:Bodies.(BodyName).NumberNodes
           NodeName = Bodies.(BodyName).NodeList(index1,:);

           z = Simulation.(BodyName).(NodeName);
           plot(z(2,:), z(3,:), 'b')
        end
    end
end
xlabel('y'),ylabel('z')
legend('Points','Nodes')
% print(fig,'Position y z','-dpng')
hold off

%%
% Graphic output: X-Z

fig = figure;
for index = 1:World.nbodies
    BodyName = BodyList{index};
    %y = yT(:,(index-1)*13+1:(index-1)*13+3)';
    %plot(y(1,:), y(3,:))  
  
    hold on
    
%     for indexP=1:length(Bodies.(BodyName).PointsList)
%         pointname=Bodies.(BodyName).PointsList{indexP};
% 
%         z=Simulation.(BodyName).(pointname);
%         plot(z(1,:), z(3,:), 'r')
%     end

    if Bodies.(BodyName).flexible % have a flexible part

        for  index1 = 1:Bodies.(BodyName).NumberNodes
           NodeName = Bodies.(BodyName).NodeList(index1,:);

           z = Simulation.(BodyName).(NodeName);
           plot(z(1,:), z(3,:), 'b')
        end
    end
end
xlabel('x'),ylabel('z')
legend('Points','Nodes')
% print(fig,'Position x z','-dpng')
hold off

%%
% Graphic output: X-Y

fig=figure;
for idx=1:World.nbodies
    BodyName=BodyList{idx};
    %index = Bodies.(BodyName).index;
    
    %y=yT(:,index:index+2)';
    
    %plot(y(1,:), y(2,:), 'r') 
    
    hold on
    
%     for indexP=1:length(Bodies.(BodyName).PointsList)
%         pointname=Bodies.(BodyName).PointsList{indexP};
% 
%         z=Simulation.(BodyName).(pointname);
%         plot(z(1,:), z(2,:), 'r')
%     end

    if Bodies.(BodyName).flexible % have a flexible part

        for  index1 = 1:Bodies.(BodyName).NumberNodes
           NodeName = Bodies.(BodyName).NodeList(index1,:);

           z = Simulation.(BodyName).(NodeName);
           plot(z(1,:), z(2,:), 'b')
        end
    end
end
xlabel('x'),ylabel('y')
legend('Points','Nodes')
% print(fig,'Position x y','-dpng')
hold off

%%
% Graphic output: Node oscillation X-Y

fig=figure;
for idx=1:World.nbodies
    BodyName=BodyList{idx};
    %index = Bodies.(BodyName).index;
    
    %y=yT(:,index:index+2)';
    
    %plot(y(1,:), y(2,:), 'r') 
    
    hold on
    
    if Bodies.(BodyName).flexible % have a flexible part

        for  index1 = 1:Bodies.(BodyName).NumberNodes
           NodeName = Bodies.(BodyName).NodeList(index1,:);
           
           y_node= Bodies.(BodyName).node.(NodeName).y_index;
           
           y=yT(:,y_node:y_node+2)';
    
           plot(y(1,:), y(2,:), 'r') 
           
        end
    end
end
xlabel('x'),ylabel('y')
legend('Points','Nodes')
% print(fig,'Position x y','-dpng')
hold off


%%
% Graphic output: Node oscillation Y-Z

fig=figure;
for idx=1:World.nbodies
    BodyName=BodyList{idx};
    %index = Bodies.(BodyName).index;
    
    %y=yT(:,index:index+2)';
    
    %plot(y(2,:), y(3,:), 'r') 
    
    hold on
    
    if Bodies.(BodyName).flexible % have a flexible part

        for  index1 = 1:Bodies.(BodyName).NumberNodes
           NodeName = Bodies.(BodyName).NodeList(index1,:);
           
           y_node= Bodies.(BodyName).node.(NodeName).y_index;
           
           y=yT(:,y_node:y_node+2)';
    
           plot(y(2,:), y(3,:), 'r') 
           
        end
    end
end
xlabel('y'),ylabel('z')
legend('Points','Nodes')
% print(fig,'Position x y','-dpng')
hold off

%%
% Graphic output: Node oscillation X-Z

fig=figure;
for idx=1:World.nbodies
    BodyName=BodyList{idx};
    %index = Bodies.(BodyName).index;
    
    %y=yT(:,index:index+2)';
    
    %plot(y(1,:), y(3,:), 'r') 
    
    hold on
    
    if Bodies.(BodyName).flexible % have a flexible part

        for  index1 = 1:Bodies.(BodyName).NumberNodes
           NodeName = Bodies.(BodyName).NodeList(index1,:);
           
           y_node= Bodies.(BodyName).node.(NodeName).y_index;
           
           y=yT(:,y_node:y_node+2)';
    
           plot(y(1,:), y(3,:), 'r') 
           
        end
    end
end
xlabel('x'),ylabel('z')
legend('Points','Nodes')
% print(fig,'Position x y','-dpng')
hold off

%%
% Velocities

fig=figure;
for idx=1:World.nbodies
    BodyName=BodyList{idx};
    %index = Bodies.(BodyName).index;
    
    %y=yT(:,index:index+2)';
    
    %plot(y(1,:), y(3,:), 'r') 
    
    hold on
    
    if Bodies.(BodyName).flexible % have a flexible part

        for  index1 = 1:Bodies.(BodyName).NumberNodes
           NodeName = Bodies.(BodyName).NodeList(index1,:);
           
           y_node= Bodies.(BodyName).node.(NodeName).y_index;
           
           y=yT(:,y_node:y_node+3)';
           leny=length(y);
           Y=zeros(Bodies.(BodyName).NumberNodes,leny);
           
           for i=1:leny
               
                %Y(i)=y(:,i)'*y(:,i);
                Y(index1,i)=y(3,i);
           end
                
           
        end
        
       
    end
end
xlabel('x'),ylabel('z')
legend('Points','Nodes')
% print(fig,'Position x y','-dpng')
hold off

figure;
plot(T,Y(1,:),'r')

figure;
plot(T,Y(end-1,:),'r')

figure;
plot(T,Y(end,:),'r')
 
figure;
contour(Y(:,max(end-100,1):end))
colorbar()
xlabel('iteration-100'),ylabel('variation')

figure
plot(sum(Y'))
xlabel('iteration'),ylabel('variation sum')


%% Error on the system constrains

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

save('Graph.mat','World','BodyList','Bodies','Simulation');

