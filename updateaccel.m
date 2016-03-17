function u_d=updateaccel(t,y)
%Function that writes q for the fourth project
%   

global World % used to describe the world 
global BodyList % List with body identifiers
global JointList % List with dynamic constrains identifiers
global Bodies % Structure with every rigid bodies in the system
global Joints % Structure with dynamics constrains

global Simulation

global TT

% y=[r;p;r_d;wP];
nbodies=World.nbodies; % number of bodies in the system 
njoints=World.njoints; % number of joints in the system


count = 1;

 %TT= [TT y]; 
 
World.y=y;

for indexE=1:nbodies
    BodyName=BodyList{indexE};
    
    Bodies.(BodyName).r  = y(count  :count+2); 
    Bodies.(BodyName).p  = y(count+3:count+6);
    Bodies.(BodyName).r_d= y(count+7:count+9);
    Bodies.(BodyName).wP = y(count+10:count+12);

    count = count+13;
    
    Bodies.(BodyName).n=[0;0;0];
    
    if  Bodies.(BodyName).flexible
              
        for  index1=1:Bodies.(BodyName).NumberNodes
           NodeName=Bodies.(BodyName).NodeList(index1,:);
           
           Bodies.(BodyName).node.(NodeName).deltaP   = y(count  :count+2 );  % equilibrium positions
           Bodies.(BodyName).node.(NodeName).omegaP   = y(count+3:count+5 );  % equilibrium orientation
           Bodies.(BodyName).node.(NodeName).deltaP_d = y(count+6:count+8 );  % 
           Bodies.(BodyName).node.(NodeName).omegaP_d = y(count+9:count+11);  %    
    
            if t> .03
                Bodies.(BodyName).node.(NodeName).f=[0;0;0];
                Bodies.(BodyName).node.(NodeName).n=[0;0;0];
            end
           
           count=count+12;
        end
              
        Bodies.(BodyName).f= Bodies.(BodyName).g;
        
    else
        Bodies.(BodyName).f= Bodies.(BodyName).g;
    end    
end

Ke = 0; % Kinetic energy
Pe = 0; % potential energy

for indexE=1:nbodies 
    
    BodyName=BodyList{indexE};

    p_mag=sqrt(Bodies.(BodyName).p'*Bodies.(BodyName).p); 
    
    Bodies.(BodyName).p=Bodies.(BodyName).p/p_mag;
    Bodies.(BodyName).A=A(Bodies.(BodyName).p);
    Bodies.(BodyName).w=Bodies.(BodyName).A*Bodies.(BodyName).wP;
    Bodies.(BodyName).A_d=skew(Bodies.(BodyName).w)*Bodies.(BodyName).A; 
    Bodies.(BodyName).geo.J=Bodies.(BodyName).A*Bodies.(BodyName).geo.JP*Bodies.(BodyName).A';
    
    
    % update: Points
    for indexP=1:length(Bodies.(BodyName).PointsList)
        pointname=Bodies.(BodyName).PointsList{indexP};
        
        Bodies.(BodyName).Points.(pointname).sP=Bodies.(BodyName).A*Bodies.(BodyName).Points.(pointname).sPp; 
        Bodies.(BodyName).Points.(pointname).rP=Bodies.(BodyName).r+Bodies.(BodyName).Points.(pointname).sP;        
        Bodies.(BodyName).Points.(pointname).sP_d=skew(Bodies.(BodyName).w)*Bodies.(BodyName).Points.(pointname).sP;
        Bodies.(BodyName).Points.(pointname).rP_d=Bodies.(BodyName).r_d+Bodies.(BodyName).Points.(pointname).sP_d; 
    end

    % update: Vectors
    for indexV=1:length(Bodies.(BodyName).VectorsList)
        nomeVector=Bodies.(BodyName).VectorsList{indexV};
        
        
        sP=Bodies.(BodyName).Vectors.(nomeVector).sP; 
        Bodies.(BodyName).Vectors.(nomeVector).s=Bodies.(BodyName).A*sP; 
        Bodies.(BodyName).Vectors.(nomeVector).s_d=Bodies.(BodyName).A_d*sP; 
    end
    
    % update: Nodes
    if  Bodies.(BodyName).flexible      
        
        for  index1=1:Bodies.(BodyName).NumberNodes
            
           NodeName=Bodies.(BodyName).NodeList(index1,:);
           
           Bodies.(BodyName).node.(NodeName).bP = Bodies.(BodyName).node.(NodeName).xP + Bodies.(BodyName).node.(NodeName).deltaP; % equilibrium positions
           d_d = Bodies.(BodyName).r_d+Bodies.(BodyName).A*skew(Bodies.(BodyName).w)*Bodies.(BodyName).node.(NodeName).bP+Bodies.(BodyName).A*Bodies.(BodyName).node.(NodeName).deltaP_d;
           alphaP = Bodies.(BodyName).wP + Bodies.(BodyName).node.(NodeName).omegaP_d; 
           Bodies.(BodyName).node.(NodeName).d_d = d_d;
           Bodies.(BodyName).node.(NodeName).alphaP = alphaP;
           
           sP=Bodies.(BodyName).A*Bodies.(BodyName).node.(NodeName).bP; 
           Bodies.(BodyName).node.(NodeName).rP=Bodies.(BodyName).r+sP; 

           mk  = Bodies.(BodyName).node.(NodeName).m;
           muk = Bodies.(BodyName).node.(NodeName).mu;           
           
           Ke = Ke+mk*(d_d'*d_d) + muk*(alphaP'*alphaP);
           
        end
    else
        Ke = Ke+Bodies.(BodyName).geo.m*Bodies.(BodyName).r_d'*Bodies.(BodyName).r_d;
    end     
end

% Save Energy

World.Ke = 0.5*Ke; % kinetic energy
World.Pe = 0.25*Pe;% potencial energy
   
% Inersia tensor Update

g=zeros(World.Msize,1);      % external forces
s=zeros(World.Msize,1);      % quadratic velocity term
q=zeros(World.Msize,1);  % body velocities
q_d=zeros(World.Msize,1);  % body velocities

for indexE=1:nbodies 
    BodyName=BodyList{indexE};
    
    index = Bodies.(BodyName).index; % body index in the mass matrix
    
    wP = Bodies.(BodyName).wP;
    
    np=skew(Bodies.(BodyName).wP)*Bodies.(BodyName).geo.JP*Bodies.(BodyName).wP;
    
    
    % external forces

    g(index:index+5)=[Bodies.(BodyName).f;Bodies.(BodyName).n+np];
    
    
    q_d(index:index+5)=[ Bodies.(BodyName).r_d;Bodies.(BodyName).wP];
    
       
    if Bodies.(BodyName).flexible % have a flexible part
        
        q(index:index+5)=[ Bodies.(BodyName).r ; Bodies.(BodyName).wP];
        
        m_rr=0;
        
        R  =  Bodies.(BodyName).A;
        
        M1 =  zeros(3);
        M2 =  zeros(3);
        M3 =  zeros(3);
        
        s1 = zeros(3,1);
        s2 = zeros(3,1);
        
        for  index1 = 1:Bodies.(BodyName).NumberNodes
           NodeName = Bodies.(BodyName).NodeList(index1,:);
           
           y_node= Bodies.(BodyName).node.(NodeName).y_index;
           
           bPk = Bodies.(BodyName).node.(NodeName).bP;
           mk  = Bodies.(BodyName).node.(NodeName).m;
           muk  = Bodies.(BodyName).node.(NodeName).mu;
           deltaP_d = Bodies.(BodyName).node.(NodeName).deltaP_d;
           % Mrr : Rigid mass contribution
           
           M1 = M1 + mk*R*skew(bPk);
           M2 = M2 + mk*skew(bPk)*R';
           M3 = M3 - mk*skew(bPk)'*skew(bPk)+muk*eye(3);
           m_rr = m_rr + mk;
           
           % Mrf:
           World.M(index:index+2,y_node:y_node+2)    = mk*R;
           World.M(index+3:index+5,y_node:y_node+2)  = mk*skew(bPk);
           World.M(index+3:index+5,y_node+3:y_node+5)= muk*eye(3);
           
           % Mfr:
           World.M(y_node:y_node+2,index:index+2)    = mk*R';
           World.M(y_node:y_node+2,index+3:index+5)  = -mk*skew(bPk);
           World.M(y_node+3:y_node+5,index+3:index+5)= muk*eye(3);
           
           % Mff:
           World.M(y_node:y_node+2,y_node:y_node+2)    = mk*eye(3);
           World.M(y_node+3:y_node+5,y_node+3:y_node+5)= muk*eye(3);
           
           % nodes local coordentes
           q(y_node:y_node+2)  = Bodies.(BodyName).node.(NodeName).deltaP;
           q(y_node+3:y_node+5)= Bodies.(BodyName).node.(NodeName).omegaP;
           
           % nodes local velocities
           q_d(y_node:y_node+2)  = Bodies.(BodyName).node.(NodeName).deltaP_d;
           q_d(y_node+3:y_node+5)= Bodies.(BodyName).node.(NodeName).omegaP_d;
           
           % external forces

           g(y_node:y_node+2)  =  Bodies.(BodyName).node.(NodeName).f;
           g(y_node+3:y_node+5)= Bodies.(BodyName).node.(NodeName).n+np;
          
           % quadratic velocity term: nodes
           
           s(y_node:y_node+2)  =-mk*skew(wP)*skew(wP)*bPk-2*mk*skew(wP)*R'*deltaP_d;
           s(y_node+3:y_node+5)= [0;0;0];
           
           % quadratic velocity term: tmp for body
           s1= s1-mk*R*skew(wP)*skew(wP)*bPk-2*mk*R*skew(wP)*deltaP_d;
           s2= s2-mk*skew(bPk)*skew(wP)*skew(wP)*bPk-2*mk*skew(bPk)*skew(wP)*deltaP_d;
        end

        World.M(index:index+2,index:index+2)    = m_rr*eye(3);
        World.M(index:index+2,index+3:index+5)  = - M1 ;
        World.M(index+3:index+5,index:index+2)  =   M2 ;
        World.M(index+3:index+5,index+3:index+5)=   M3 ;
        
        % quadratic velocity term: body
        s(index:index+2)  = s1;
        s(index+3:index+5)= s2;
    
    else
        World.M(index:index+2,index:index+2) = Bodies.(BodyName).geo.m*eye(3);
        World.M(index+3:index+5,index+3:index+5)= Bodies.(BodyName).geo.JP; 
    end
    
end

% TMP: Used for debugging
World.g=g;
World.s=s;
World.q=q;
World.q_d=q_d;

if njoints > 0
    % Update Joints information
    nlines=0;

    for indexJ=1:njoints 
        Jointname=JointList{indexJ};
        
        % Body is rigedly fixed in the space
        if strcmp(Joints.(Jointname).type, 'Fix')
            Joints.(Jointname).constr=zeros(6,1);
            Joints.(Jointname).Di=eye(6);
            Joints.(Jointname).f=zeros(6,1);

            Joints.(Jointname).numlin=6;

            nlines=nlines+6;
        end

        % Spherical Joint linking two rigid bodies 
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

        % Revolute joint linking two rigid bodies
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
    D=zeros(nlines,World.Msize);
    
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
    
    Phid = D*q_d;  % Evaluate velocity constraints

    gamma=gamma -2*World.alpha*Phid-World.beta^2*Phi;   %Baumgarte Method
    
    World.gamma=gamma;
    World.Phi=Phi;
    World.Phid=Phid;
    
    World.Z=[World.M,D';D,zeros(World.size,World.size)];
    
    if World.Flexible
        %World.F=[World.g+World.s-Bodies.C1.G*World.q;World.gamma]; % Buggy.....
        World.F=[World.g+World.s-Bodies.C1.G*World.q;World.gamma];
        %World.F=[World.g;World.gamma];
        
    else
        World.F=[World.g;World.gamma]; 
    end
    
else
    World.Z=World.M;
    if World.Flexible
        World.F=World.g+World.s-Bodies.C1.G*World.q;  % Buggy.....
    else
        World.F=World.g;
    end
end

%q_d=World.Z\World.F;

Msis=World.Z'*World.Z;
q_d=(1e-15*eye(size(Msis))+Msis)\World.Z'*World.F;

u_d=[];


for indexE=1:nbodies
    BodyName=BodyList{indexE};
    index = Bodies.(BodyName).index;

    Bodies.(BodyName).r_dd=q_d(index:index+2); 
    Bodies.(BodyName).w_d=q_d(index+3:index+5);
    
    Bodies.(BodyName).p_d=0.5*Local(Bodies.(BodyName).p)'*Bodies.(BodyName).wP;
    
    u_d=[u_d; Bodies.(BodyName).r_d; Bodies.(BodyName).p_d; Bodies.(BodyName).r_dd; Bodies.(BodyName).w_d]; 
    
    if Bodies.(BodyName).flexible % is a flexible body
           
        for index1=1:Bodies.(BodyName).NumberNodes
            NodeName = Bodies.(BodyName).NodeList(index1,:);
            
            idx = Bodies.(BodyName).node.(NodeName).y_index;
            
            deltaP_dd = q_d(idx:idx+2);
            omegaP_dd = q_d(idx+3:idx+5);
            
            Bodies.(BodyName).node.(NodeName).delta_dd=deltaP_dd; 
            Bodies.(BodyName).node.(NodeName).omega_dd=omegaP_dd;
                   
            deltaP_d= Bodies.(BodyName).node.(NodeName).deltaP_d; 
            omegaP_d= Bodies.(BodyName).node.(NodeName).omegaP_d;
            
            u_d=[u_d; deltaP_d; omegaP_d ; deltaP_dd; omegaP_dd ];
            
        end
    end
   %TT= [TT u_d]; 
end
end

