function  GraphMovie( file)
%   Detailed explanation goes here
%    Draw solution to the spheres packing problem
%    arguments:
%               solution - vector with box dimensions and sphere coordinates
%               val      - image title value
%               label    - image label
%               spheres  - vector with the sphere ratios 
%               file     - file name to print the image 

load(file) 

figure
set(gca,'nextplot','replacechildren');
set(gcf,'Renderer','zbuffer');
light('Position',[20 30 30],'Style','infinite');

writerObj=VideoWriter(file);
writerObj.Quality=100;
writerObj.FrameRate=5;

open(writerObj);


[x,y,z] = sphere;

hold on
%title( sys.Title)
shading flat;
camproj perspective;
%campos([-xbox -ybox 2*zbox]);
camtarget([0 0 5]);
%camup([1 0 0])
camva(10);
view(3);

%view(axes,[63.5 18]);

cameratoolbar
%az=120;
%el=45;
%view(az,el);


axis('square',[-.5 2 -.5 2.5 0 2]);
axis equal;


%%
% init dots on the screen

    i=1;
    
    
    for indexE=1:World.nbodies 
        BodyName=BodyList{indexE};
        
        if World.ElasticNet

                num_frames=length(Simulation.(BodyName).r(1,:));
                
                R = Simulation.(BodyName).r(:,1);

                Sx=R(1);
                Sy=R(3);
                Sz=R(2);

                r=.1;

                s(i)=surf(r*x+Sx,r*y+Sy,r*z+Sz);  % sphere centered at (x,y,z)
                set(s(i),'EdgeColor','none');
                set(s(i),'FaceLighting','phong','FaceColor','interp','AmbientStrength',0.1);

                i=i+1;
                
               
        else
            for  index1 = 1:Bodies.(BodyName).NumberNodes
                
                num_frames=length(Simulation.(BodyName).(NodeName)(1,:));

                NodeName = Bodies.(BodyName).NodeList(index1,:);

                R = Simulation.(BodyName).(NodeName)(:,1);

                Sx=R(1);
                Sy=R(3);
                Sz=R(2);

                r=.1;

                s(i)=surf(r*x+Sx,r*y+Sy,r*z+Sz);  % sphere centered at (x,y,z)
                set(s(i),'EdgeColor','none');
                set(s(i),'FaceLighting','phong','FaceColor','interp','AmbientStrength',0.1);

                i=i+1;
            end
        end
        
        for indexP=1:length(Bodies.(BodyName).PointsList)
            
            PointName = Bodies.(BodyName).PointsList{indexP};
            
            R = Simulation.(BodyName).(PointName)(:,1);
            
            Sx=R(1);
            Sy=R(3);
            Sz=R(2);

            r=.09;
            
            s(i)=surf(r*x+Sx,r*y+Sy,r*z+Sz);  % sphere centered at (x,y,z)
            set(s(i),'EdgeColor','none');
            set(s(i),'FaceLighting','phong','FaceColor','interp','AmbientStrength',0.1);
            
            i=i+1;
        end 
        
        
        
    end

        
num_frames
for f=2:1:num_frames
    
    i=1;
     for indexE=1:World.nbodies  
         BodyName=BodyList{indexE};        

        
        if World.ElasticNet

                R = Simulation.(BodyName).r(:,f);

                Sx=R(1);
                Sy=R(3);
                Sz=R(2);

                r=.1;

                s(i)=surf(r*x+Sx,r*y+Sy,r*z+Sz);  % sphere centered at (x,y,z)
                set(s(i),'EdgeColor','none');
                set(s(i),'FaceLighting','phong','FaceColor','interp','AmbientStrength',0.1);

                i=i+1;
        else        
            for  index1 = 1:Bodies.(BodyName).NumberNodes
                NodeName = Bodies.(BodyName).NodeList(index1,:);

                R = Simulation.(BodyName).(NodeName)(:,f);

                Sx=R(1);
                Sy=R(3);
                Sz=R(2);

                r=.1;

                %set(s(i+1),'xdata',r*Rot+Sx,'ydata',r*y+Sy,'zdata',r*z+Sz);  % sphere centered at (x,y,z)
                set(s(i),'xdata',r*x+Sx,'ydata',r*y+Sy,'zdata',r*z+Sz);  % sphere centered at (x,y,z)
                set(s(i),'EdgeColor','none');
                set(s(i),'FaceLighting','phong','FaceColor','interp','AmbientStrength',0.1);

                i=i+1;
            end
        end
        
        for indexP=1:length(Bodies.(BodyName).PointsList)
            
            PointName = Bodies.(BodyName).PointsList{indexP};
            
            R = Simulation.(BodyName).(PointName)(:,f);
            
            Sx=R(1);
            Sy=R(3);
            Sz=R(2);

            r=.15;
            
            set(s(i),'xdata',r*x+Sx,'ydata',r*y+Sy,'zdata',r*z+Sz);  % sphere centered at (x,y,z)
            set(s(i),'EdgeColor','none');
            set(s(i),'FaceLighting','phong','FaceColor','interp','AmbientStrength',0.1);
            
            i=i+1;
        end  
    end
    
    drawnow;
    
    frame=getframe(gcf);
    writeVideo(writerObj,frame);
    %pause(.1);
end
close(writerObj);
hold on
end

