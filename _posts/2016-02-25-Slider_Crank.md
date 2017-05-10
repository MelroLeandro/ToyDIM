---
layout:     post
title:        3D Slider-crank 
author:     Mellean
tags: 	matlab  dynamic template
subtitle:   A toy system
category:  ToyDIM
visualworkflow: true
---
{% if page.visualworkflow == true %}
   {% include workflowmatlab.html %}
{% endif %}   
   

<!--
This HTML was auto-generated from MATLAB code.
To make changes, update the MATLAB code and republish this document.
-->
<pre class="codeinput">
<span class="comment">%------------------------------Multibody Dynamic--------------------------%</span>
<span class="comment">%------------------------------A toy system-------------------------------%</span>
<span class="comment">% Problem:  3D slider-crank</span>
<span class="comment">% Autor: Mellean</span>
<span class="comment">% Data: 25Fev16</span>
<span class="comment">% Version:</span>
<span class="comment">%-------------------------------------------------------------------------%</span>
clc
clear <span class="string">all</span>
<span class="comment">%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%</span>
<span class="comment">% Glabal variables used to</span>
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


<span class="keyword">global</span> TT Err Ke Pe
</pre>

<h2>World parameters <a name="1"></a></h2>

<pre class="codeinput">World.gravity=[0;0;0]; <span class="comment">% force to be applied on each body</span>

World.ElasticNet = false;  <span class="comment">% force are generated using and elastic network</span>
World.Regularistion = true;
World.RFactor = 1e-15;     <span class="comment">% regularization factor</span>
World.FMNcontact = false;
World.FMNdensity = false;

<span class="comment">% Contact</span>
World.n=1.5;
World.e=0.6;
World.k=2*10^4;
World.mu=0.74;

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
JointList={<span class="string">'Fix'</span>,<span class="string">'J1'</span>,<span class="string">'J2'</span>,<span class="string">'J3'</span>,<span class="string">'Slide'</span>}; <span class="comment">% list of dynamic constrains</span>
Motors={'Chank'};

<span class="comment">%Ground: body C1</span>
<span class="comment">%Geometry</span>

Bodies.Ground.geo.m=1; <span class="comment">% mass</span>
Bodies.Ground.flexible=false;

<span class="comment">% Inertia tensor for body Ground</span>
Bodies.Ground.geo.JP=diag([1,1,1]);

<span class="comment">% Points in body C1</span>
Bodies.Ground.PointsList={<span class="string">'P1'</span>,<span class="string">'A'</span>};
Bodies.Ground.Points.P1.sPp=[0,0.5,0]'; <span class="comment">% local frame coordinates</span>
Bodies.Ground.Points.A.sPp=[0,5,0]';  <span class="comment">% B is a reference point used on the slicer</span>

<span class="comment">% Vectors in body C1</span>
Bodies.Ground.VectorsList={<span class="string">'V0'</span>,<span class="string">'V1'</span>,<span class="string">'V2'</span>};
Bodies.Ground.Vectors.V0.sP=[1,0,0]'; <span class="comment">% local frame coordinates</span>
Bodies.Ground.Vectors.V1.sP=[0,1,0]';
Bodies.Ground.Vectors.V2.sP=[0,0,1]';

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
Bodies.C1.VectorsList={<span class="string">'V0'</span>,<span class="string">'V1'</span>,<span class="string">'V2'</span>};
Bodies.C1.Vectors.V0.sP=[1,0,0]';
Bodies.C1.Vectors.V1.sP=[0,1,0]';
Bodies.C1.Vectors.V2.sP=[0,0,1]';

<span class="comment">% Body initial values</span>
Bodies.C1.r=[0,0.5697,-0.4951]';
Bodies.C1.r_d=[0,0.1330,0.0187]';
Bodies.C1.r_dd=[0,-0.0057,0.0356]';
Bodies.C1.p=[-0.7548,0.6560,0,0]';
Bodies.C1.p_d=[-0.0881,-0.1014,0,0]';
Bodies.C1.w=[0.2686,0,0]';
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

Bodies.C2.Points.P2.sPp=[0,-2.5,0]';
Bodies.C2.Points.P3.sPp=[0,2.5,0]';

<span class="comment">%List of vectors in body C3</span>
Bodies.C2.VectorsList={<span class="string">'V0'</span>,<span class="string">'V1'</span>,<span class="string">'V2'</span>};
Bodies.C2.Vectors.V0.sP=[1,0,0]';
Bodies.C2.Vectors.V1.sP=[0,1,0]';
Bodies.C2.Vectors.V2.sP=[0,0,1]';

<span class="comment">% Body initial values</span>
Bodies.C2.r=[0,3.0898,-0.4951]';
Bodies.C2.r_d=[0,0.2698,0.0187]';
Bodies.C2.r_dd=[0,-0.0044,0.0356]';
Bodies.C2.p=[0.9950,0.0995,0,0]';
Bodies.C2.p_d=[0.0004,-0.0038,0,0]';
Bodies.C2.w=[-0.0076,0,0]';
Bodies.C2.wp=[0,0,0]';
Bodies.C2.np=[0,0,0]';
Bodies.C2.exists=true;

<span class="comment">%Body C3 a Rod</span>
<span class="comment">%%Geometry</span>
Bodies.C3.geo.m=3; <span class="comment">% rod mass</span>
Bodies.C3.geo.h=1; <span class="comment">% rod length</span>
Bodies.C3.geo.r=1; <span class="comment">% rod radius</span>
Bodies.C3.flexible=false;

<span class="comment">% Inertia tensor for body C3</span>
Bodies.C3.geo.JP=diag([1/12*(Bodies.C3.geo.m*(3*Bodies.C3.geo.r^2+Bodies.C3.geo.h^2)),1/2*(Bodies.C3.geo.m*Bodies.C3.geo.r^2),<span class="keyword">...</span>
     1/12*(Bodies.C3.geo.m*(3*Bodies.C3.geo.r^2+Bodies.C3.geo.h^2))]);

<span class="comment">%List of points in body C3</span>
Bodies.C3.PointsList={<span class="string">'P3'</span>,<span class="string">'A'</span>};

Bodies.C3.Points.P3.sPp=[0,-0.5,0]';
Bodies.C3.Points.A.sPp=[0,1,0]';
<span class="comment">%List of vectors in body C3</span>
Bodies.C3.VectorsList={<span class="string">'V0'</span>,<span class="string">'V1'</span>,<span class="string">'V2'</span>};
Bodies.C3.Vectors.V0.sP=[-1,0,0]';
Bodies.C3.Vectors.V1.sP=[0,-1,0]';
Bodies.C3.Vectors.V2.sP=[0,0,-1]';
Bodies.C3.fLL=[0,0,0]';

<span class="comment">% Body initial values</span>
Bodies.C3.r=[0,6.0403,0]';
Bodies.C3.r_d=[0,0.2735,0]';
Bodies.C3.r_dd=[0,0.0026,0]';
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

<span class="comment">%Sherical joint linking body C1 and body C2</span>
Joints.J1.type=<span class="string">'Rev2'</span>;  <span class="comment">% spherical joint between body_1 and body_2</span>
Joints.J1.body_1=<span class="string">'Ground'</span>; <span class="comment">% body_1 identifier</span>
Joints.J1.body_2=<span class="string">'C1'</span>; <span class="comment">% body_2 identifier</span>
Joints.J1.point=<span class="string">'P1'</span>;  <span class="comment">% point identifier</span>
Joints.J1.vector1=<span class="string">'V0'</span>; <span class="comment">% vector identifier</span>
Joints.J1.vector11=<span class="string">'V1'</span>; <span class="comment">% vector identifier</span>
Joints.J1.vector12=<span class="string">'V2'</span>; <span class="comment">% vector identifier</span>

<span class="comment">% Revolute joint linking body C2 and body C3</span>
Joints.J2.type=<span class="string">'Rev2'</span>;  <span class="comment">% revolute joint between body_1 and body_2</span>
Joints.J2.body_1=<span class="string">'C1'</span>; <span class="comment">% body_1 identifier</span>
Joints.J2.body_2=<span class="string">'C2'</span>; <span class="comment">% body_2 identifier</span>
Joints.J2.point=<span class="string">'P2'</span>;  <span class="comment">% point identifier</span>
Joints.J2.vector1=<span class="string">'V0'</span>; <span class="comment">% vector identifier</span>
Joints.J2.vector11=<span class="string">'V1'</span>; <span class="comment">% vector identifier</span>
Joints.J2.vector12=<span class="string">'V2'</span>; <span class="comment">% vector identifier</span>

<span class="comment">% Revolute joint linking body C2 and body C3</span>
Joints.J3.type=<span class="string">'Rev2'</span>;  <span class="comment">% revolute joint between body_1 and body_2</span>
Joints.J3.body_1=<span class="string">'C2'</span>; <span class="comment">% body_1 identifier</span>
Joints.J3.body_2=<span class="string">'C3'</span>; <span class="comment">% body_2 identifier</span>
Joints.J3.point=<span class="string">'P3'</span>;  <span class="comment">% point identifier</span>
Joints.J3.vector1=<span class="string">'V0'</span>; <span class="comment">% vector identifier</span>
Joints.J3.vector11=<span class="string">'V1'</span>; <span class="comment">% vector identifier</span>
Joints.J3.vector12=<span class="string">'V2'</span>; <span class="comment">% vector identifier</span>

<span class="comment">% Slide linking body C3 and Ground</span>
Joints.Slide.type=<span class="string">'FixSlide'</span>;  <span class="comment">% fix body in the space</span>
Joints.Slide.body_1=<span class="string">'C3'</span>; <span class="comment">% body identifier</span>

 Motor.Chank.type=<span class="string">'linear'</span>;
 Motor.Chank.body=<span class="string">'C1'</span>;
 Motor.Chank.Point=<span class="string">'P2'</span>;
 Motor.Chank.force=[0,0,0.001]';

<span class="comment">% Tansport Multibody system information</span>


World.nbodies = length(BodyList);  <span class="comment">% number of bodies in the system</span>
World.njoints = length(JointList); <span class="comment">% number of joints in the system</span>
World.nmotors = length(Motors); <span class="comment">% number of motors in the system</span>
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
t=20.00; <span class="comment">% Final time</span>
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

252 successful steps
0 failed attempts
1513 function evaluations

timeode45 =

    4.7749
</pre>

<h2>Output <a name="6"></a></h2>

<p>Graphic output: Orthogonal projections of each body</p>

<pre class="codeinput">fig=figure;
<span class="keyword">for</span> index=1:nbodies
    BodyName=BodyList{index};
    plot(yT(:,(index-1)*13+2), yT(:,(index-1)*13+3))

    hold <span class="string">on</span>
<span class="keyword">end</span>
xlabel(<span class="string">'y'</span>),ylabel(<span class="string">'z'</span>)
legend(<span class="string">'position y'</span>,<span class="string">'position z'</span>)
<span class="comment">% print(fig,'Position y z','-dpng')</span>
hold <span class="string">off</span>

fig=figure;
<span class="keyword">for</span> index=1:nbodies
    BodyName=BodyList{index};
    plot(yT(:,(index-1)*13+1), yT(:,(index-1)*13+3))
    hold <span class="string">on</span>
<span class="keyword">end</span>
xlabel(<span class="string">'x'</span>),ylabel(<span class="string">'z'</span>)
legend(<span class="string">'position x'</span>,<span class="string">'position z'</span>)
<span class="comment">% print(fig,'Position x z','-dpng')</span>
hold <span class="string">off</span>

fig=figure;
<span class="keyword">for</span> index=1:nbodies
    BodyName=BodyList{index};
    plot(yT(:,(index-1)*13+1), yT(:,(index-1)*13+2))
    hold <span class="string">on</span>
<span class="keyword">end</span>
xlabel(<span class="string">'x'</span>),ylabel(<span class="string">'y'</span>)
legend(<span class="string">'position x'</span>,<span class="string">'position y'</span>)
<span class="comment">% print(fig,'Position x z','-dpng')</span>
hold <span class="string">off</span>

fig=figure;
<span class="keyword">for</span> index=1:nbodies
    BodyName=BodyList{index};
    plot(yT(:,(index-1)*13+1), yT(:,(index-1)*13+2))
    hold <span class="string">on</span>
<span class="keyword">end</span>
xlabel(<span class="string">'x'</span>),ylabel(<span class="string">'y'</span>)
legend(<span class="string">'position x'</span>,<span class="string">'position y'</span>)
<span class="comment">% print(fig,'Position x z','-dpng')</span>
hold <span class="string">off</span>
</pre><img vspace="5" hspace="5" src="data:image/png;base64, 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" alt=""> 
<img vspace="5" hspace="5" src="data:image/png;base64, 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" alt="">
 <img vspace="5" hspace="5" src="data:image/png;base64, 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" alt=""> 
<img vspace="5" hspace="5" src="data:image/png;base64, iVBORw0KGgoAAAANSUhEUgAAAgAAAAGACAIAAABUQk3oAAAACXBIWXMAAAxOAAAMTgF/d4wjAAAAB3RJTUUH4QUKAQQM2onuiwAAACR0RVh0U29mdHdhcmUATUFUTEFCLCBUaGUgTWF0aFdvcmtzLCBJbmMuPFjdGAAAACJ0RVh0Q3JlYXRpb24gVGltZQAxMC1NYXktMjAxNyAwMjowNDoxMu1Gt0cAAAraSURBVHic7d3bdto6GEZRtEfe/5W9Lxh1XSBAsGLp9zfnFUnTWCagZWQObVmWCwB5/hs9AADGEACAUAIAEEoAAEIJAEAoAQAIJQAAoQQAIJQAAIQSAIBQAgAQSgAAQgkAQCgBAAglAAChBAAglAAAhBIAgFACABBKAABCCQBAKAEACCUAAKEEACDU1+gB/Exr7XphWZaxIwGorlIAWmvrvL+9DMAHLAEBhBIAgFDFFlKenANY/wlgTrPNt6c6B3D8lTvqVMSQ7Ubt7Kjt2tlzb/f4jT5nCQgglAAAhDrVOYBa+wJEmXCOqnQO4DLfKRSAuooFADjGhGcsi5r5sFUAgMdmnrmqmLyjTgIDhBIAgFACABBKAABCCQBAKAEACCUAwAndPP/y4dMxJ3+O5gG8DgB4y2GzZZeXH7zzIgYvdBAA4C3Hz5bbI/TtW8G/+Z2bLx9+c9uAm99zfeue+x+7GWHpz6kVAGBeN9Pr/YT7cApeJ+71l9zPzjc/8/D33F+4H979MApxDgAo7Dr5Pj9O3/PL3xxAxdn/IgBAdcuy3BzO8yYBAObV/niyCn+98GYDundie6qgHOcAgHk9Wbi/X+6/P+u7fuf+dO7Nzzz8PS/d/PdyC0ECAFRyP8m+8537b778X9svv5vZ3/mZmVkCAgglAMCkKh5T1yIAAKEEACCUAACEEgCAUAIAEKpSANq/Rg8HmJfPA3hHpReCPXzjVuAYPg/gfCoFYFXxJddQnc8DeDhCnwcA8Csm/zyA6uoF4M33BTxwRJxTawOOefmp7UH6kM8DeGnm9ep6AXjCvA+BZjhIfzLL3y9VzaPSs4Auo//GwMEm/zyA9fPIis5Lp3oEAJzM5J8H8NMfno0AAJVM9XkAdY/9r4oFoPR1DZzMx48bJlEsAECOElNqiUF+RwCAxyZ81gp9CQDwQOkDW95U7GmgAPQiAAChBAAglAAAhBIAgFACABBKAABCCQBAKAEACCUAAKEEACCUAACEEgCAUAIAEEoAAEIJAEAoAQAIJQAAoQQAIJQAAIQSAIBQAgAQ6mv0AH6mtXa9sCzL2JEAVFcpAK21dd7fXgbgA1WXgMz+ADtVDQAAO1VaArq8OgfgDAEwlXVSmlOxADw/B2DeB6aynZQmjIElIIBQAgAQqtIS0LIsVvkBeqkUgIt5H6AfS0AAoQQAIJQAAIQSAIBQAgAQSgAAQgkAQCgBAAglAAChBAAglAAAhBIAgFACABBKAABCCQBAKAEACCUAAKEEACCUAACEEgCAUAIAEEoAAEIJAEAoAQAIJQAAob5GD+AHWmvbL5dlGTUSgBOoFICLSR+gH0tAAKHqBaC1drMWBMAHii0Btdauq0DrhZt/vV6wUgTMYPKj1UoBeDmtm/fpyK2J/baT0oQxqLcEBEAXlQIwYT8B6iq2BGSVH6CXSgG4mPcB+qm0BARARwIAEEoAAEIJAEAoAQAIJQAAoQQAIJQAAIQSAIBQAgAQSgAAQgkAQCgBAAglAAChBAAglAAAhBIAgFACABBKAABCCQBAKAEACCUAAKEEACCUAACEEgCAUCUD0FobPQSA8koGAID96gXA4T9AF8UC0FpblmX0KADO4Gv0AHpaHxyIBPu1dnE7YqfJVywqBeDl4b95H5jKdlKaMAb1loCuV+KEVyVALZUeAawtdSYAYL9ijwAA6KVkABz+A+xXMgAA7CcAAKEEACCUAACEEgCAUAIAEEoAAEIJAEAoAQAIJQAAoQa8GdzDN/L07g4ABxsQgIdv6ukNPgEONnIJaFkWb+sPMIpzAAChBp8DuC77WP8BON7IcwBPvgPAb7MEBBBKAABCCQBAKAEACCUAAKEEACCUAACEEgCAUANeCPaxmzcO8vIxgD0qBWA743sXOYCdSi4Bee8ggP3qBcDsD9BFpSWgy5+Vn+8asK4LKQQwg8kXq4sF4PkniJn3galMfuay3hIQAF1UCsCE/QSoq9IS0PYzhK32AOxUKQAX8z5AP5WWgADoSAAAQgkAQCgBAAglAAChBAAglAAAhBIAgFACABBKAABCCQBAKAEACCUAAKEEACCUAACEEgCAUAIAEEoAAEIJAEAoAQAIJQAAoQQAIJQAAIQSAIBQAgAQ6mv0AH6mtXa9sCzL2JEAVFcpAK21dd7fXgbgA5aAAEJVCoBDfoCOKi0Brb5b/3GGAJjKOinNqV4Anqz+m/fpyK2J/baT0oQxqLQEdHHuF6CfSgEw+wN0VGwJaPsYSgwA9qgUADM+QEeVloAA6EgAAEIJAEAoAQAIJQAAoQQAIJQAAIQSAIBQAgAQSgAAQgkAQCgBAAglAAChBAAglAAAhBIAgFACABBKAABCCQBAKAEACCUAAKEEACCUAACEEgCAUAIAEKpeAFpro4cAcAbFAmD2B+ilUgBaa8uyjB4FwElUCoDZH6Cjr9ED6GldIJIK9mvt4nbETpOvWp8qAOZ9YCrbSWnCGFRaAgKgIwEACFVpCWh9AHW9YMEHYI9KATDjA3RkCQgglAAAhBIAgFACABBKAABCCQBAKAEACCUAAKEEACCUAACEEgCAUAIAEEoAAEIJAEAoAQAIJQAAoQQAIJQAAIQSAIBQAgAQSgAAQgkAQCgBAAglAAChBAAgVL0AtNZGDwHgDIoFoLW2LIsGAOxXKQDX2f9yuWgAwH6VAgBAR1+jB9DT+rDg+kABYKzJ1ypOFQDzPh25NbHfdlKaMAaWgABCVQrAeu53PRsMwMcqBeDypwFmf4D9igXgYqEfoJN6AQCgCwEACCUAAKEEACCUAACEEgCAUAIAEEoAAEIJAEAoAQAIJQAAoQQAIJQAAIQSAIBQAgAQSgAAQgkAQCgBAAglAAChBAAglAAAhBIAgFACABBKAABCCQBAqHoBaK2NHsJfowYzZLtROztqu3b2xNudULEA+MsB9FIpAK21ZVlGjwLgJCoFwOwP0NG8x9Q3qz3rOL97HGB1CJjcbPPt1+gBfOun19Rs1yzA5CotAQHQkQAAhJr3HMC9784KAPCBSgEAoKOTLAEd8BSgl5tof5xgu2/+tnPs7GcjOWATo25RP/qxXhs9093ns5EMcYYAHPPHW5blyYauP3DVcTxDtvtyo79k1JX8wUgO2MSoW9RvmHZnj7xF/d4v36N8AA54efC6iYPvOT/dbpfr4f2Ndj9CHHIlDxnJ/Leo7rPwOxu96nV3nvAWNaHyAZj2mj23IbfpqL/1wJ2decKqaOYrc94Xgj10wBOBPtvE9ijjs1F9vGt7tjvqiVU7txs1Q4Xs7M67z2f2322rKxaAA/5IH0/fL9+s4je2u/2PH2x3/85+Zs9/D5kQr47f2VFX7867z2f2322rK78ExBDrEyeGnE48cosDjdrZUX9cjicAr62PE2/ukL999xiy3Xc2uj5x4tLvMdk72z1mQvxuJAds4rd3dsgfN+ruU07546nDVrG/uxltH0L+xhiGbPflRh/+2G9v98gzFkc+u2z98nrht3d2yB836u5zP4btl1NNueUDMNDAR+jHbzdqZ0eJupKjdnZarguAUM4BAIQSAIBQAgAQSgAAQgkAQCgBAAglAAChBAD+unkzhoEjgQMIAPx1wBsBwTzcyuGW2Z8QHgEAhBIA+Ecb8bHpMIQAwF/zfJI4HMBaJ0AojwAAQgkAQCgBAAglAAChBAAglAAAhBIAgFACABBKAABCCQBAKAEACPU/T3kAqodLpxMAAAAASUVORK5CYII=" alt="">
 <p>Graphic output:Velocity of body 4 c-of-m</p><pre class="codeinput">indexE=4;
figure; plot(T, yT(:,(indexE-1)*13+9));
xlabel(<span class="string">'T'</span>),ylabel(<span class="string">'slider y_d [m/s]'</span>)
<span class="comment">%print(fig,'velocity x','-dpng')</span>
hold <span class="string">off</span>
</pre><img vspace="5" hspace="5" src="data:image/png;base64, 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" alt="">
 <p>Graphic output: Position of body 4 c-of-m</p><pre class="codeinput">indexE=4;
figure; plot(T, yT(:,(indexE-1)*13+2));
xlabel(<span class="string">'T'</span>),ylabel(<span class="string">'slider y [m]'</span>)
<span class="comment">%print(fig,'position y','-dpng')</span>
hold <span class="string">off</span>
</pre><img vspace="5" hspace="5" src="data:image/png;base64, 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"  alt=""> 

<p>Graphic output:Velocity of body 3 c-of-m</p><pre class="codeinput">figure; plot(Simulation.C3.vel(2,:)')
xlabel(<span class="string">'Interation'</span>),ylabel(<span class="string">'slider y_d [m/s]'</span>)
</pre><img vspace="5" hspace="5" src="data:image/png;base64, iVBORw0KGgoAAAANSUhEUgAAAgAAAAGACAIAAABUQk3oAAAACXBIWXMAAAxOAAAMTgF/d4wjAAAAB3RJTUUH4QUKAQQNrY7eHQAAACR0RVh0U29mdHdhcmUATUFUTEFCLCBUaGUgTWF0aFdvcmtzLCBJbmMuPFjdGAAAACJ0RVh0Q3JlYXRpb24gVGltZQAxMC1NYXktMjAxNyAwMjowNDoxM5pBh9EAAA4fSURBVHic7d1Rs5pIEAZQ2Mr//8vuAxWKoCLCIN3T59Q+JK7XyBX7G3oGGB+PxwBAPf/d/QYAuIcAAChKAAAUJQAAihIAAEUJAICiBABAUQIAoCgBAFCUAAAoSgAAFCUAAIoSAABFCQCAogQAQFECAKAoAQBQlAAAKEoAABQlAACKEgAARQkAgKIEAEBRAgCgqD93v4F/jOP4eDw2/u/yrxvPBOCjQAEwVf+NDFg+vgoDAL4VpQU01/0pA3Y+GYDDogTAfqo/QBOBWkB7TAcHLzNAUwgILtrgNVkAzL++lxkQ7Zd7QB/HN7YiDlsRR8BBar4WEABNRAmAee53FfXLzAyYnwB5xTqwelf9l52f1SPvfhYglIA1KtYcwOq383xaQLRfH0BeUVpA76j4ABeJHgAAXEQAABQlAACKEgAARcVaBXTSt+cJmGCGj3ytOtZVAHy7573bs+3BlLJd4n2tOtZVAHzr3R652oPtuPTkuUC33cN9rRIpHQDvrHbNece1y5LUsvjetRv7WgUkAD6bd9Bpl7W/kkKEor/B1yoCAfCFaR81ciGydPunr9WNBMDXliMXOytxpB5K+1rdQgAc93gYtnC/znZCX6tfEgCnGLZwry53PF+rnxEAkE+RMfJ0NND3Nt7LpSDaWB63wqWmmjj91z3frEsJgGamPdXOynWmHaxC3V+SAdcRAC1NgzI7K1eYB/4F+VpdRABAAgUH/isy4AoCoD17Kg3VbPu85JfQnFVAEJfSz6UcAVzCQQDnqf5cTQBcRQZwhurPDwiAC8kAjlH9+Q0BALGo/vyMALiWgwAgLAEAgRj+80sCAKJQ/fkxAXA5XSD2UP35PQEA91P9uYUA+AUHAUBAAgBuZvjPXQTAjzgIAKIRAABFxQqA8dMgefzrN+8Hrqb/w40CBcA4jo/HY6O4T0+YZMwAXSAglCgBMBX3YRiSFnc4wPCfe0UJgD0evisA7aS8I9h8uPD8+PQHUQFEELyfkS8A3lX/Qd0nFf2fCpZFKWAYZGoBDZvVPwXzwEAcUQJgnvtdlfhlZmav/jAz/CeCQC2gKQOeq//ywWUeCAOAMwIFwPBU01eRoOLTB8N/gojSAnqnv6JvGgAIInoAAHARAQBQlAC4gS5QZSYAiEMAABQlAACKEgAARQkA+B0TAIQiAACKEgD3sBCoIMN/ohEAAEUJAICiBAD8gv4PAQkAgKIEAEBRAgCgKAEAUJQAuI1TAYB7CQC4nCVAxCQAAIoSAABFCQCAogQAXMsEAGEJgDtZCATcSAAAFCUA4EL6P0QmAACKEgA3Mw0A3EUAABQlAACKEgBwFTPABCcAAIoSAABFCQCAogQAQFGxAmDcsSR+z3NycSoAcItAATCO4+Px2K7v/VV/emUJEPFFCYCp+g/DsJEB83MAOC9KAOyh+gM09OfuN9DSfOggKoAIgnetuwoAdR8IZVmUAoZBphYQZGEGmBSiBMA897ua6Q2YmQB9iBIAw98MeK7+cwaM4zg/IhgAToo1B7Bq4q8iQYsfoKFARwAvFSn6TgbuiQkAsogeAABcRAAAFCUAAIoSAABFCQBoyQwwiQgAgKIEAEBRAiAKpwIAPyYAAIoSAABFCQCAogQANGMNKLkIAICiBEAgFgIBvyQAAIoSAABFCQCAogQAtGEJEOkcuSfwxxuyF7mPI0BqB28Kv1HiP8YDABFoAQEUdSQAtjs8+j9nOBUgKRMAZOQIAKCoBgEwN/11/wESORsA4zhOPZ/pDzIAIIs2LaA5BgDIwhwAQFFnA2Bq+yy7QC3eFWRiCRBJNTgCmIu+6g+QyMEAGMfRfC9AaqcuBTFlgIE/QEanWkCPx8PST4Ck2swB6AgBpHOwBbSiI9TQdDkgv0Xgai3PA5g6Qg1f8CWHGgBNNDgCWFXkSzNgvuCE4wyCcLhGXmcD4Je1eP63ZADAeWcDYLUKSFEGyCLTEcBHcxTFeUtAZcHnLJPNAWxT94FQlkUpYBg0aAE1eR8A/Fimy0HP8w2h+k5UZgkQqR0JgO0DmUsPc6z/AWgl0xHARPUHaOLgHEDA2QwAvnIkAIzBr+ZyQCn4jMguXwsIgCYEAEBRAgCgKAEAUJQAAChKAAAUJQDgCGtA6UCbewJvnBfmpIFjnAoAXK3lTeGHp8u0OWEYICwtIICiBABAUW1aQDO3CAbIonEADIo+QBINbgr//KAMaMJCIOBSze4JvFz/46Zd9E0w04dmk8Cr7j8AwVkFBFBUyzmAqe2j/wOQQrM5gI1HAAhICwi+YwaYbggAgKIEQGjTqQAAVxAAAEUJAPiCCQB6IgAAimoQAE4ABsjobAA47QsgKS0ggKLOBoBrwAEk1eZaQO4CRgWWANGZ9tcCAiCFNquAno8DaMXJwMBFWq4COj8fsOfHxQxAE4FWAU1Zsl3fVX+AVqIEwHwksZEBzjkAaKjBJPByAuDSAq36AzR0NgCGQ3V5NcZvVdnnlxUVNGcNKAcE71ofDICNrdpTfC8q0Oo+EMqyKAUMg4MBMG/Vqi8fcAs7MK0ElW5AW1EmgZdzCRIF4AeiBMDwNwOeq/+cAcszzgQDwEnNVgHNfz35as8vvjzR7MyLw2FacHTpnlVAQV48EdMAQHOBWkAA/FLjANCaB8jieAtoz6V7oAOab/TqeAC8nJuVBwBZNLgl5MZfAQjLJDBAUQIgDbcGA9qyCgi2mAGmY+2vBgpACg1WAS0JBoAsDraA3q32sQoIIAuTwABFNQiA5eWaz78aAL9xNgDmyzW7MgRALm1aQKsbuXARpwL8mDWg9M0cAEBRDa4FtOoCtXhXAFyuwRGAWzYCZKQFBFCUAEjGPPDPmAGmewIAoCgBAFCUAAAoSgDkYxrgB0wAUIEAAChKAMCa4T9FCACAogQAQFECICXzwMB5AgD+YQKAOgQAQFECAKAoAQBQlAAAKCpWAHy8p/z412/eT2QWAl3BDDClBAqA6Y6SG8V9esJEBgCcFCUA5vsJK+7cxfCfaqIEwB7uOQzQ0J9b/tXVGP/byj4fLrx72SJRMU0D1NjWy/lNcoXg/Yx7AuBMgX5X/U++LEBzy6IUMAwytYCGzepfk7VAwGFRAmCe+12V+GVmqv5cRP+Hmu5pAb00ZcBz9V8+uMwDYQBwRqAAGJ5q+ioSVHyAhqK0gN5R9D8yDXCS/g9lRQ8AuJTqT2UCAKAoAdADXaBjDP8pTgAAFCUAOuEg4FuG/yAAAIoSAP1wELDTOBr+wzBEOxEMrqb0w8wRQFccBAD7CQAKMfyHJQFAFao/rAiA3kxdII2gFdUfngmADj0eJgP+ofrDSwKAzqn+8I4A6JaDAOv9YZsA6FnlDJhKv+oPG5wI1rkpA+rUwTnw6mwyHCYA+lcnA4psJrSiBVRChV6Q6g/fcgRQxZwBnVVJPR84TAAUMpXIbmKgmw2BuwiAcuYYSFo6DfmhFQFQ1HJWIH4lXU5gxH+3kIUAqGuupDHLa8x3BT0RAPxTXm8su6t1Soo+XE0A8I93YfDyCce8W5Cq4sOPCQDeelmRz59PoNBDEAKA7yjf0A1nAgMUJQAAihIAAEUJAICiBABAUQIAoKhYy0DHcXy8X2Y4/rsEfeOZAHwUKACm6r+RAcvHx+7vbwJwsSgtoLnuTxmw88kAHBYlAPZT/QGauKcFdLibP/3guwyYX1ZCABEEb1bfEwCHC/T8gy8zQN0HQgk+c5mvBQRAE1ECYJ77XQ3tl5kZMD8B8gq0DPR5DegqEpYLhHR7AE4KFADDU1l/jgR1H6CVKC2gd1R8gItEDwAALiIAAIoSAABFCQCAogQAQFECAKAoAQBQlAAAKEoAABQlAACKEgAARQkAgKIEAEBRAgCgKAEAUJQAAChKAAAUJQAAihIAAEUJAICiBABAUQIAoCgBAFCUAAAoSgAAFCUAAIoSAABFCQCAogQAQFECAKAoAQBQlAAAKEoAABQlAACKihUA4zg2fFpGfWyarYjDVrAhUACM4/h4PHzSAL8RJQCm6j8Mw8cMkBAATUQJgJ3mnADgpD+3/KurUXyrmt7HwYGtiMNWxNHHVkRzTwAcq/jbw39HBgBfydcCmgYChgMAJwVqqc+rgJZv6eWo30wAwHmBjgBeVv/BYB/gGtGH0gb7ABfppLwmzYl3q6ESbc7zW93zSDTLd5j0Q5nf9nYHNddWZPws9r/nCFtxzyqgtl5OHmTxboYjxeY8d+ee33z8zXneinQfyvMvfEj4WbzcinSfxXMTewj8WQSaAzhm/ynEKSTanI0Rzfzm42/Onm9g/K14lvGz2CPRVqySeAj5WaQPgA7Ma1tziTn++ta7rcj1ofgsorl9aL+TALjZtKPcPhBgKe+HkqXubHtuleT6LBItX+xhDiCvDr6r/cn7ofRX/ZNuTorJ6okjAOhB/FqzRx9bkUj6AHieV0nk5fqTvJuzMceVaHMyfijPbyzjZ/FyWcHqOSm2YvVI5M8i6C/xW7f/Hg/buXw7oFzrnd95uRXpPhSfRSiJzsm4/x0AcIv0LSAAjhEAAEUJAICiBABAUQIAoChnApPbyyV3wB6OAMjt3UWDl1pdleXl66S45Au8JADgFEce5CUA6Mp8hv3yXhyrR+a/tnrO6g18/CkIwpnApLe6evCwuIrA6g8vnz9fmOXlkzde592r7X9luJdJYHqzp8i+vGLX6g+txuyKPmEJACrafxvIwTQv/TIHQCHPpfxjcbfyh445AiC3PddVX/Xil/cXfPdTyx9ZPnn5Os//+p5XhjjMSgEUpQUEUJQAAChKAAAUJQAAihIAAEUJAICiBABAUQIAoCgBAFCUAAAoSgAAFPU/87OyCjiL9kAAAAAASUVORK5CYII=" alt=""> 

<p>Graphic output: acceleration of body 3 c-of-m</p><pre class="codeinput">figure; plot(Simulation.C3.acc(2,:)')
xlabel(<span class="string">'Interation'</span>),ylabel(<span class="string">'slider y_dd [m/s]'</span>)
</pre><img vspace="5" hspace="5" src="data:image/png;base64, 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" alt=""> 

<p>Graphic output: Error on thr system constrains</p><pre class="codeinput">fig=figure;plot( Err)
xlabel(<span class="string">'iteration'</span>),ylabel(<span class="string">'Error'</span>)
</pre><img vspace="5" hspace="5" src="data:image/png;base64, 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" alt=""> 

<p>Graphic output:  System kinetic Energy</p><pre class="codeinput">fig=figure; plot(Ke)
xlabel(<span class="string">'iteration'</span>),ylabel(<span class="string">'Ke'</span>)

<span class="comment">% save('Graph.mat','yT','ListaCorpos','Corpo');</span>
</pre><img vspace="5" hspace="5" src="data:image/png;base64, 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"  alt="">
 <p class="footer"><br><a href="http://www.mathworks.com/products/matlab/">Published with MATLAB&reg; R2014a</a><br></p></div>
