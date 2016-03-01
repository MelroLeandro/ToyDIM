function u_d=updateaccel(t,y)
% Autor: Carlos Leandro
% Data: 25Fev16
% Version:

global World % used to describe the world 
global BodyList % List with body identifiers
global JointList % List with dynamic constrains identifiers
global Bodies % Structure with every rigid bodies in the system
global Joints % Structure with dynamis constrains 

% y=[r;p;r_d;wp];
stiffness=World.stiffness;
nbodies=length(BodyList); % number of bodies in the system 
njoints=length(JointList); % number of joints in the system

Ke = 0; % Kinetic energy
Pe = 0; % potential energy

for indexE=1:nbodies
    BodyName=BodyList{indexE};
    Bodies.(BodyName).r=y((indexE-1)*13+1:(indexE-1)*13+3); 
    Bodies.(BodyName).p=y((indexE-1)*13+4:(indexE-1)*13+7);
    Bodies.(BodyName).r_d=y((indexE-1)*13+8:(indexE-1)*13+10);
    Bodies.(BodyName).wp=y((indexE-1)*13+11:(indexE-1)*13+13);
    
    Bodies.(BodyName).n=[0;0;0];
    Bodies.(BodyName).f=[0;0;0];% Bodies.(BodyName).g;
    
    % kinetic energy
    Ke = Ke+Bodies.(BodyName).geo.m*Bodies.(BodyName).r_d'*Bodies.(BodyName).r_d;
    
    % total potential energy in a network
    if World.ElasticNet && Bodies.(BodyName).coupled
        xi=Bodies.(BodyName).r;
        xi0=Bodies.(BodyName).r0;
        for idx=1:Bodies.(BodyName).num
            BodyName2=Bodies.(BodyName).Nei{idx};
            xj=Bodies.(BodyName2).r;
            xj0=Bodies.(BodyName2).r0;
            dx = xj-xi;
            dx0 = xj0-xi0;
            mdx = dx'*dx;
            mdx0 = dx0'*dx0;
            km=mdx*mdx0;
            Pe = Pe + km;
            norma=sqrt(mdx);
            if norma>1e-10
                Bodies.(BodyName).f = Bodies.(BodyName).f + stiffness*km*dx/norma;
            end
        end
        Pe=0;
    end
end

World.Ke = 0.5*Ke;
World.Pe = 0.25*Pe;

for indexE=1:nbodies 
    BodyName=BodyList{indexE};

    p_mag=sqrt(Bodies.(BodyName).p'*Bodies.(BodyName).p);    
    Bodies.(BodyName).p=Bodies.(BodyName).p/p_mag;
    Bodies.(BodyName).A=A(Bodies.(BodyName).p);
    Bodies.(BodyName).w=Bodies.(BodyName).A*Bodies.(BodyName).wp;
    Bodies.(BodyName).A_d=skew(Bodies.(BodyName).w)*Bodies.(BodyName).A; 
    Bodies.(BodyName).geo.J=Bodies.(BodyName).A*Bodies.(BodyName).geo.JP*Bodies.(BodyName).A';
    
    
    % update 
    for indexP=1:length(Bodies.(BodyName).PointsList)
        pointname=Bodies.(BodyName).PointsList{indexP};
        
        Bodies.(BodyName).Points.(pointname).sP=Bodies.(BodyName).A*Bodies.(BodyName).Points.(pointname).sPp; 
        Bodies.(BodyName).Points.(pointname).rP=Bodies.(BodyName).r+Bodies.(BodyName).Points.(pointname).sP;        
        Bodies.(BodyName).Points.(pointname).sP_d=skew(Bodies.(BodyName).w)*Bodies.(BodyName).Points.(pointname).sP;
        Bodies.(BodyName).Points.(pointname).rP_d=Bodies.(BodyName).r_d+Bodies.(BodyName).Points.(pointname).sP_d; 
    end

    for indexV=1:length(Bodies.(BodyName).VectorsList)
        nomeVector=Bodies.(BodyName).VectorsList{indexV};
        
        
        sP=Bodies.(BodyName).Vectors.(nomeVector).sP; 
        Bodies.(BodyName).Vectors.(nomeVector).s=Bodies.(BodyName).A*sP; 
        Bodies.(BodyName).Vectors.(nomeVector).s_d=Bodies.(BodyName).A_d*sP; 
    end
end

    
% Inersia tensor Update
d=[];
q_dot=[];
for indexE=1:nbodies 
    BodyName=BodyList{indexE};
    
    np=skew(Bodies.(BodyName).wp)*Bodies.(BodyName).geo.JP*Bodies.(BodyName).wp;
    % acceleration right side

    d=[d;Bodies.(BodyName).f;Bodies.(BodyName).n+np];
    
    index=(indexE-1)*6+1;
    World.M(index:index+2,index:index+2)=Bodies.(BodyName).geo.m*eye(3);
    World.M(index+3:index+5,index+3:index+5)=Bodies.(BodyName).geo.J;
    q_dot=[q_dot; Bodies.(BodyName).r_d;Bodies.(BodyName).wp];
end
World.d=d;
World.q_dot=q_dot;

if njoints > 0
    % Update Joints information
    nlines=0;

    for indexJ=1:njoints 
        Jointname=JointList{indexJ};

        if strcmp(Joints.(Jointname).type, 'Fix')
            Joints.(Jointname).constr=zeros(6,1);
            Joints.(Jointname).Di=eye(6);
            Joints.(Jointname).f=zeros(6,1);

            Joints.(Jointname).numlin=6;

            nlines=nlines+6;
        end

        if strcmp(Joints.(Jointname).type, 'She')

            Ci=Joints.(Jointname).body_1;
            Cj=Joints.(Jointname).body_2;
            P=Joints.(Jointname).point;

            c=Bodies.(Cj).Points.(P).rP-Bodies.(Ci).Points.(P).rP; 
            World.C1=[World.C1 norm(c)];

            Joints.(Jointname).constr=c; %

            Qri=-eye(3,3);
            Qpi=skew(Bodies.(Ci).Points.(P).sP)*Bodies.(Ci).A;
            Qrj=eye(3,3);
            Qpj=-skew(Bodies.(Cj).Points.(P).sP)*Bodies.(Cj).A;

            Joints.(Jointname).Di=[Qri,Qpi]; 
            Joints.(Jointname).Dj=[Qrj,Qpj]; 

            Joints.(Jointname).f=skew(Bodies.(Ci).w)*Bodies.(Ci).Points.(P).sP_d...
            -skew(Bodies.(Cj).w)*Bodies.(Cj).Points.(P).sP_d; 

            Joints.(Jointname).numlin=3;

            nlines=nlines+3;

        end

        if strcmp(Joints.(Jointname).type, 'Rev')

            C1=Joints.(Jointname).body_1;
            C2=Joints.(Jointname).body_2;
            P1=Joints.(Jointname).point;
            V1=Joints.(Jointname).vector;

            c1=Bodies.(C2).Points.(P1).rP-Bodies.(C1).Points.(P1).rP; 
            c2=skew(Bodies.(C1).Vectors.(V1).s)*Bodies.(C2).Vectors.(V1).s;

            Joints.(Jointname).constr=([c1;c2]); %

            Qri1=-eye(3,3);
            Qpi1=skew(Bodies.(C1).Points.(P1).sP)*Bodies.(C1).A;
            Qrj1=eye(3,3);
            Qpj1=-skew(Bodies.(C2).Points.(P1).sP)*Bodies.(C2).A;

            Di1=[Qri1,Qpi1];
            Dj1=[Qrj1,Qpj1]; 

            Qri2=zeros(3,3);
            Qpi2=skew(Bodies.(C2).Vectors.(V1).s)*skew(Bodies.(C1).Vectors.(V1).s)*Bodies.(C1).A;
            Qrj2=zeros(3,3);
            Qpj2=-skew(Bodies.(C1).Vectors.(V1).s)*skew(Bodies.(C2).Vectors.(V1).s)*Bodies.(C2).A;


            Di2=[Qri2,Qpi2]; 
            Dj2=[Qrj2,Qpj2]; 

            Joints.(Jointname).Di=[Di1;Di2];
            Joints.(Jointname).Dj=[Dj1;Dj2];

            f1=skew(Bodies.(C1).w)*Bodies.(C1).Points.(P1).sP_d...
            -skew(Bodies.(C2).w)*Bodies.(C2).Points.(P1).sP_d;

            f2=-2*skew(Bodies.(C1).Vectors.(V1).s_d)*Bodies.(C2).Vectors.(V1).s_d...
            +skew(Bodies.(C2).Vectors.(V1).s)*skew(Bodies.(C1).w)*Bodies.(C1).Vectors.(V1).s_d...
            -skew(Bodies.(C1).Vectors.(V1).s)*skew(Bodies.(C2).w)*Bodies.(C2).Vectors.(V1).s_d;

            Joints.(Jointname).f=[f1;f2];
            Joints.(Jointname).numlin=6;

            nlines=nlines+6;

        end    
    end

    % Jacobian
    D=zeros(nlines,nbodies*6);
    Phi=zeros(nlines,1);

    cont=1;
    for indexJ=1:njoints
       Jointname=JointList{indexJ};

        D(cont:cont+Joints.(Jointname).numlin-1,...
            Bodies.(Joints.(Jointname).body_1).index:Bodies.(Joints.(Jointname).body_1).index+5)=Joints.(Jointname).Di;
        if ~strcmp(Joints.(Jointname).type, 'Fix')
            D(cont:cont+Joints.(Jointname).numlin-1,...
                Bodies.(Joints.(Jointname).body_2).index:Bodies.(Joints.(Jointname).body_2).index+5)=Joints.(Jointname).Dj;
        end

        Phi(cont:cont+Joints.(Jointname).numlin-1)=Joints.(Jointname).constr;

        cont=cont+Joints.(Jointname).numlin;
    end

    Err=sqrt(Phi'*Phi); % error
    
    World.Err= [World.Err Err];
    
    World.D=D;
    World.size=nlines;
    
    gamma=zeros(World.size,1);
    cont=1;

    for indexJ=1:njoints
        Jointname=JointList{indexJ};
        gamma(cont:cont+Joints.(Jointname).numlin-1,1)=Joints.(Jointname).f; 

        cont=cont+Joints.(Jointname).numlin;
    end
    
    Phid = D*q_dot;  % Evaluate velocity constraints

    gamma=gamma -2*World.alpha*Phid-World.beta^2*Phi;   %Baumgarte Method
    
    World.gamma=gamma;
    World.Phi=Phi;
    World.Phid=Phid;
    
    World.Z=[World.M,D';D,zeros(World.size,World.size)];
    World.F=[World.d;World.gamma];  
    
else
    World.Z=World.M;
    World.F=World.d; 
end

%q_d=World.Z\World.F;

Msis=World.Z'*World.Z;
q_d=(1e-10*eye(size(Msis))+Msis)\World.Z'*World.F;

u_d=[];

for indexE=1:nbodies
    BodyName=BodyList{indexE};
        
    Bodies.(BodyName).r_dd=q_d((indexE-1)*6+1:(indexE-1)*6+3); 
    Bodies.(BodyName).w_d=q_d((indexE-1)*6+4:(indexE-1)*6+6); 
    
    Bodies.(BodyName).p_d=0.5*Local(Bodies.(BodyName).p)'*Bodies.(BodyName).wp;  
    u_d=[u_d; Bodies.(BodyName).r_d; Bodies.(BodyName).p_d; Bodies.(BodyName).r_dd; Bodies.(BodyName).w_d]; 
    
end
end

