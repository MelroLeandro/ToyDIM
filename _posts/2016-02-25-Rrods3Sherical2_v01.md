---
layout:     post
title:        3D Pendulum 
author:     Mellean
tags: 	matlab  dynamic template spherical-joints  pendulum
subtitle:    3 rods linked  by spherical joints
category:  ToyDIM
visualworkflow: true
---
---
{% if page.visualworkflow == true %}
   {% include workflowmatlab.html %}
{% endif %}   
   

<!--
This HTML was auto-generated from MATLAB code.
To make changes, update the MATLAB code and republish this document.
-->

<pre class="codeinput"><span class="comment">%-------------------------------------------------------------------------%</span>
<span class="comment">%------------------------------Multibody Dynamic--------------------------%</span>
<span class="comment">%------------------------------A toy system-------------------------------%</span>
<span class="comment">% Problem: 3 rods linked using two spherical joints 3D</span>
<span class="comment">% Author: Mellean</span>
<span class="comment">% Data: 25Fev16</span>
<span class="comment">% Version: 1.0</span>
<span class="comment">%-------------------------------------------------------------------------%</span>
clc
clear <span class="string">all</span>
<span class="comment">%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%</span>
<span class="comment">% Global variables used to</span>
<span class="comment">%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%</span>
addpath(<span class="string">'../dynamic'</span>)

<span class="keyword">global</span> World <span class="comment">% used to describe the world</span>
<span class="keyword">global</span> BodyList <span class="comment">% List with body identifiers</span>
<span class="keyword">global</span> JointList <span class="comment">% List with dynamic constrains identifiers</span>
<span class="keyword">global</span> Bodies <span class="comment">% Structure with every rigid bodies in the system</span>
<span class="keyword">global</span> Joints <span class="comment">% Structure with dynamics constrains</span>
<span class="keyword">global</span> Simulation
<span class="keyword">global</span> Motors
<span class="keyword">global</span> Motor

<span class="keyword">global</span> TT Err
</pre>
<h2>World parameters <a name="1"></a></h2>
<pre class="codeinput">World.gravity=[0;0;-0.00981]; <span class="comment">% force to be applied on each body</span>
World.ElasticNet = false;  <span class="comment">% force are generated using and elastic network</span>
World.Regularistion = true;
World.RFactor = 1e-15;     <span class="comment">% regularization factor</span>
World.FMNcontact = false;
World.FMNdensity = false;

<span class="comment">% Baumgarte Method</span>
World.alpha=5.0;
World.beta=5.0;

<span class="comment">% Debugging information</span>

World.C1=[];
World.Err=[];

<span class="comment">% Flexible Multibody system</span>
World.Flexible=false;

</pre>
<h2>Multibody system configuration <a name="2"></a></h2>

<pre class="codeinput">
BodyList={<span class="string">'Ground'</span>,<span class="string">'C1'</span>,<span class="string">'C2'</span>,<span class="string">'C3'</span>}; <span class="comment">% list of system bodies</span>
JointList={<span class="string">'Fix'</span>,<span class="string">'J1'</span>,<span class="string">'J2'</span>,<span class="string">'J3'</span>}; <span class="comment">% list of dynamic constrains</span>
Motors={};

<span class="comment">%Ground: body C1</span>
<span class="comment">%Geometry</span>

Bodies.Ground.geo.m=1; <span class="comment">% massa</span>
Bodies.Ground.flexible=false;


Bodies.Ground.geo.JP=diag([1,1,1]); <span class="comment">% Moment of inertia</span>

<span class="comment">% Points in body Ground</span>
Bodies.Ground.PointsList={<span class="string">'P1'</span>};
Bodies.Ground.Points.P1.sPp=[0,0.5,0]'; <span class="comment">% local frame coordinates</span>

<span class="comment">% Vectors in body Ground</span>
Bodies.Ground.VectorsList={};

<span class="comment">% Body initial values</span>
Bodies.Ground.r=[0,0.0,0]'; <span class="comment">% Body initial position in global coordinates</span>
Bodies.Ground.r_d=[0,0,0]';  <span class="comment">% initial velocity</span>
Bodies.Ground.r_dd=[0,0,0]';  <span class="comment">% initial acceleration</span>
Bodies.Ground.p=[1,0,0,0]';  <span class="comment">% initial Euler parameters</span>
Bodies.Ground.p_d=[0,0,0,0]';<span class="comment">% Euler parameters derivative</span>
Bodies.Ground.w=[0,0,0]';    <span class="comment">% initial angular velocity</span>
Bodies.Ground.wp=[0,0,0]';  <span class="comment">% initial angular acceleration</span>
Bodies.Ground.np=[0,0,0]';   <span class="comment">% initial moment</span>
Bodies.Ground.exists=true;

<span class="comment">%Body C1 a Rod</span>
<span class="comment">%%Geometry</span>
Bodies.C1.geo.m=1; <span class="comment">% rod mass</span>
Bodies.C1.geo.h=1; <span class="comment">% rod length</span>
Bodies.C1.geo.r=1; <span class="comment">% rod radius</span>
Bodies.C1.flexible=false;

<span class="comment">% Inertia tensor for body C1</span>
Bodies.C1.geo.JP=diag([1/12*(Bodies.C1.geo.m*(3*Bodies.C1.geo.r^2+Bodies.C1.geo.h^2)),1/2*(Bodies.C1.geo.m*Bodies.C1.geo.r^2),<span class="keyword">...</span>
     1/12*(Bodies.C1.geo.m*(3*Bodies.C1.geo.r^2+Bodies.C1.geo.h^2))]);

<span class="comment">%List of points in body C1</span>
Bodies.C1.PointsList={<span class="string">'P1'</span>,<span class="string">'P2'</span>};

Bodies.C1.Points.P1.sPp=[0,-0.5,0]';
Bodies.C1.Points.P2.sPp=[0,0.5,0]';

<span class="comment">%List of vectors in body C1</span>
Bodies.C1.VectorsList={};

<span class="comment">% Body initial values</span>
Bodies.C1.r=[0,1,0]';
Bodies.C1.r_d=[0,0,0]';
Bodies.C1.r_dd=[0,0,0]';
Bodies.C1.p=[1,0,0,0]';
Bodies.C1.p_d=[0,0,0,0]';
Bodies.C1.w=[0,0,0]';
Bodies.C1.wp=[0,0,0]';
Bodies.C1.np=[0,0,0]';
Bodies.C1.exists=true;

<span class="comment">%Body C2 a Rod</span>
<span class="comment">%%Geometry</span>
Bodies.C2.geo.m=1; <span class="comment">% rod mass</span>
Bodies.C2.geo.h=1; <span class="comment">% rod length</span>
Bodies.C2.geo.r=1; <span class="comment">% rod radius</span>
Bodies.C2.flexible=false;

<span class="comment">% Inertia tensor for body C2</span>
Bodies.C2.geo.JP=diag([1/12*(Bodies.C2.geo.m*(3*Bodies.C2.geo.r^2+Bodies.C2.geo.h^2)),1/2*(Bodies.C2.geo.m*Bodies.C2.geo.r^2),<span class="keyword">...</span>
     1/12*(Bodies.C2.geo.m*(3*Bodies.C2.geo.r^2+Bodies.C2.geo.h^2))]);

<span class="comment">%List of points in body C2</span>
Bodies.C2.PointsList={<span class="string">'P2'</span>,<span class="string">'P3'</span>};

Bodies.C2.Points.P2.sPp=[0,-0.5,0]';
Bodies.C2.Points.P3.sPp=[0,0.5,0]';

<span class="comment">%List of vectors in body C2</span>
Bodies.C2.VectorsList={};

<span class="comment">% Body initial values</span>
Bodies.C2.r=[0,2,0]';
Bodies.C2.r_d=[0,0,0]';
Bodies.C2.r_dd=[0,0,0]';
Bodies.C2.p=[1,0,0,0]';
Bodies.C2.p_d=[0,0,0,0]';
Bodies.C2.w=[0,0,0]';
Bodies.C2.wp=[0,0,0]';
Bodies.C2.np=[0,0,0]';
Bodies.C2.exists=true;

<span class="comment">%Body C3 a Rod</span>
<span class="comment">%%Geometry</span>
Bodies.C3.geo.m=1; <span class="comment">% rod mass</span>
Bodies.C3.geo.h=1; <span class="comment">% rod length</span>
Bodies.C3.geo.r=1; <span class="comment">% rod radius</span>
Bodies.C3.flexible=false;

<span class="comment">% Inertia tensor for body C3</span>
Bodies.C3.geo.JP=diag([1/12*(Bodies.C3.geo.m*(3*Bodies.C3.geo.r^2+Bodies.C3.geo.h^2)),1/2*(Bodies.C3.geo.m*Bodies.C3.geo.r^2),<span class="keyword">...</span>
     1/12*(Bodies.C3.geo.m*(3*Bodies.C3.geo.r^2+Bodies.C3.geo.h^2))]);

<span class="comment">%List of points in body C3</span>
Bodies.C3.PointsList={<span class="string">'P3'</span>,<span class="string">'P4'</span>};

Bodies.C3.Points.P3.sPp=[0,-0.5,0]';
Bodies.C3.Points.P4.sPp=[0,0.5,0]';
<span class="comment">%List of vectors in body C3</span>
Bodies.C3.VectorsList={};

<span class="comment">% Body initial values</span>
Bodies.C3.r=[0,3,0]';
Bodies.C3.r_d=[0,0,0]';
Bodies.C3.r_dd=[0,0,0]';
Bodies.C3.p=[1,0,0,0]';
Bodies.C3.p_d=[0,0,0,0]';
Bodies.C3.w=[0,0,0]';
Bodies.C3.wp=[0,0,0]';
Bodies.C3.np=[0,0,0]';
Bodies.C3.exists=true;
</pre>

<h2>System dynamic constrains <a name="3"></a></h2>

<pre class="codeinput"><span class="comment">%Ground joint</span>

Joints.Fix.type=<span class="string">'Fix'</span>;  <span class="comment">% fix body in the space</span>
Joints.Fix.body_1=<span class="string">'Ground'</span>; <span class="comment">% body identifier</span>

<span class="comment">%Sherical joint linking body Ground and body C1</span>
Joints.J1.type=<span class="string">'She'</span>;  <span class="comment">% spherical joint between body_1 and body_2</span>
Joints.J1.body_1=<span class="string">'Ground'</span>; <span class="comment">% body_1 identifier</span>
Joints.J1.body_2=<span class="string">'C1'</span>; <span class="comment">% body_2 identifier</span>
Joints.J1.point=<span class="string">'P1'</span>;  <span class="comment">% point identifier</span>

<span class="comment">% Sherical joint linking body C1 and body C2</span>
Joints.J2.type=<span class="string">'She'</span>;  <span class="comment">% revolute joint between body_1 and body_2</span>
Joints.J2.body_1=<span class="string">'C1'</span>; <span class="comment">% body_1 identifier</span>
Joints.J2.body_2=<span class="string">'C2'</span>; <span class="comment">% body_2 identifier</span>
Joints.J2.point=<span class="string">'P2'</span>;  <span class="comment">% point identifier</span>

<span class="comment">% Sherical joint linking body C2 and body C3</span>
Joints.J3.type=<span class="string">'She'</span>;  <span class="comment">% revolute joint between body_1 and body_2</span>
Joints.J3.body_1=<span class="string">'C2'</span>; <span class="comment">% body_1 identifier</span>
Joints.J3.body_2=<span class="string">'C3'</span>; <span class="comment">% body_2 identifier</span>
Joints.J3.point=<span class="string">'P3'</span>;  <span class="comment">% point identifier</span>

<span class="comment">% Transport Multibody system information</span>

World.nbodies = length(BodyList);  <span class="comment">% number of bodies in the system</span>
World.njoints = length(JointList); <span class="comment">% number of joints in the system</span>
World.nmotors = length(Motors); <span class="comment">% number of joints in the system</span>
World.NNodes  = 0 ;            <span class="comment">% count number of nodes in the system</span>
World.Msize   = World.NNodes*6+World.nbodies*6; <span class="comment">% mass matrix size</span>


</pre>

<h2>Mass matrix assembly <a name="4"></a></h2>

<pre class="codeinput">y_d=[];

nbodies=length(BodyList);
World.M=zeros(nbodies*6);

<span class="keyword">for</span> indexE=1:nbodies
    BodyName=BodyList{indexE};
    Bodies.(BodyName).g=World.gravity;
    O=zeros(3,3);
    index=(indexE-1)*6+1;

    Bodies.(BodyName).index=index; <span class="comment">% body index in the mass matrix</span>
    Bodies.(BodyName).forca=[];
    <span class="comment">% recursive definition for the initial force</span>
    y_d=[y_d; Bodies.(BodyName).r;Bodies.(BodyName).p;Bodies.(BodyName).r_d;Bodies.(BodyName).w];
<span class="keyword">end</span>
</pre>

<h2>Start simulation <a name="5"></a></h2>

<pre class="codeinput"><span class="comment">% set integration parameters</span>
t0=0;    <span class="comment">% Initial time</span>
t=100.00; <span class="comment">% Final time</span>
step=0.001; <span class="comment">% Time-step</span>


tspan = [t0:step:t];

<span class="comment">% Set integrator and its parameters</span>

fprintf(<span class="string">'\n\n ODE45\n\n'</span>)
tol=1e-5;
options=odeset(<span class="string">'RelTol'</span>,tol,<span class="string">'Stats'</span>,<span class="string">'on'</span>,<span class="string">'OutputFcn'</span>,@odeOUT);

tic
[T, yT]= ode45(@updateaccel, tspan, y_d, options);
timeode45=toc
</pre><pre class="codeoutput">

 ODE45

878 successful steps
2 failed attempts
5281 function evaluations

timeode45 =

   14.1549

</pre>

<h2>Output <a name="6"></a></h2>

<p>Output: systems c-of-m yz-projection</p><pre class="codeinput">fig=figure;
<span class="keyword">for</span> index=1:nbodies
    BodyName=BodyList{index};
    plot(yT(:,(index-1)*13+2), yT(:,(index-1)*13+3))

    hold <span class="string">on</span>
<span class="keyword">end</span>
xlabel(<span class="string">'y'</span>),ylabel(<span class="string">'z'</span>)
<span class="comment">% print(fig,'Positionyz','-dpng')</span>

hold <span class="string">off</span>
</pre><img vspace="5" hspace="5" src="<img src="data:image/png;base64, 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" alt=""> 
<p>Output: systems c-of-m xz-projection</p><pre class="codeinput">fig=figure;
<span class="keyword">for</span> index=1:nbodies
    BodyName=BodyList{index};
    plot(yT(:,(index-1)*13+1), yT(:,(index-1)*13+3))
    hold <span class="string">on</span>
<span class="keyword">end</span>
xlabel(<span class="string">'x'</span>),ylabel(<span class="string">'z'</span>)
<span class="comment">% print(fig,'Positionxz','-dpng')</span>
hold <span class="string">off</span>
</pre><img vspace="5" hspace="5" src="data:image/png;base64, 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" alt="">
 <p>Output: Velocidades**2</p><pre class="codeinput">fig=figure;
<span class="keyword">for</span> indexE=1:nbodies
    L=yT(:,(indexE-1)*13+8:(indexE-1)*13+10);
    v=[];
    <span class="keyword">for</span> i=1:length(L)
        v=[v L(i,:)*L(i,:)'];
    <span class="keyword">end</span>
    plot(T, v)
    hold <span class="string">on</span>
<span class="keyword">end</span>
xlabel(<span class="string">'T'</span>),ylabel(<span class="string">'v^2[m/s]'</span>)
<span class="comment">%print(fig,'velocity','-dpng')</span>
hold <span class="string">off</span>
</pre><img vspace="5" hspace="5" src="data:image/png;base64, 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" alt="">
<p>Output: Error on the system constrains</p><pre class="codeinput">fig=figure;
plot( Err)
xlabel(<span class="string">'iteration'</span>),ylabel(<span class="string">'Error'</span>)
</pre><img vspace="5" hspace="5" src="data:image/png;base64, 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" alt=""> 
<p class="footer"><br><a href="http://www.mathworks.com/products/matlab/">Published with MATLAB&reg; R2014a</a><br></p></div>
