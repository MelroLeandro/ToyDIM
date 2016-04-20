function  Pic(body,BodyName,indexE,Time)
    
    global World
    global BodyList % List with body identifiers
    global Bodies

    box = body.box;
    delta = World.delta;
    
    box_1= body.A*box(1,:)'+body.r;
    box_2= body.A*box(2,:)'+body.r;
    
    id_ini=fix((World.box(2,:)'-box_2)*1/delta)+World.Min;
    id_end=fix((World.box(2,:)'-box_1)*1/delta)+World.Min;
    
    if id_ini(1)<World.Min(1)  || id_end(1)>World.Max(1) || id_ini(2)<World.Min(2)  || id_end(2)>World.Max(2) || id_ini(3)<World.Min(3)  || id_end(3)>World.Max(3)
        body.exists = false;
        return
    end
    
    % recod for each point in the box
    for x_id = id_ini(1):id_end(1) % inc x     
        
        for y_id = id_ini(2):id_end(2) % inc y          
            
            for z_id = id_ini(3):id_end(3) % inc z
                          
                World.id=[x_id y_id z_id]';
                rA=World.box(2,:)'-(World.id-World.Min)*delta;
                rAP=body.A'*(rA-body.r);  
                
                if body.exists && body.shape(rAP) 
                    
                    % intersects an existent body
                    World.BodyName=BodyName;
                    World.Time=Time;
                    %id_end(3)
                    % up contact
                    if z_id<id_end(3) && World.ecoder_x(fix(x_id))>0 && World.ecoder_y(fix(y_id))>0 && World.ecoder_z(fix(z_id+1))>0
                        for index=1:indexE-1
                            BodyNameB=BodyList{index};
                            bodyB=Bodies.(BodyNameB);
                            if Bodies.(BodyNameB).exists && Bodies.(BodyNameB).contact
                                if  bitand(World.ecoder_x(fix(x_id)),2^bodyB.ord)>0 && bitand(World.ecoder_y(fix(y_id)),2^bodyB.ord)>0 && bitand(World.ecoder_z(fix(z_id+1)),2^bodyB.ord)>0
                                    id=[x_id y_id z_id+1]';
                                    rB=World.box(2,:)'-(id-World.Min)*delta;
                                    
                                    if World.debuging
                                        fprintf('UP');
                                    end
                                    [ identation, value, P, Q] = contact( rA, rB, body, bodyB);
                                    if identation == 1
                                        World.contact=true;
                                        if value> Bodies.(BodyName).delta
                                            Bodies.(BodyName).delta=value;
                                            f=World.K*value^(3/2);
                                            af= f*bodyB.normal(rAP);
                                            rBP=bodyB.A'*(rB-bodyB.r); 
                                            bf= -f*body.normal(rBP);   
                                            Bodies.(BodyName).Tf=af;
                                            Bodies.(BodyNameB).Tf=bf;
                                            fprintf('A(%f(t=%f) -%s -- %s -> %f (%f,%f,%f)-(%f,%f,%f)\n',identation, Time, BodyName, BodyNameB, value, af(1),af(2),af(3),bf(1),bf(2),bf(3));
                                        end
                                    end
                                end
                            end
                        end
                    end
                    
                    % down contact
                    if z_id > id_ini(3) && World.ecoder_x(fix(x_id))>0 && World.ecoder_y(fix(y_id))>0 && World.ecoder_z(fix(z_id-1))>0
                        for index=1:indexE-1
                            BodyNameB=BodyList{index};
                            bodyB=Bodies.(BodyNameB);
                            if Bodies.(BodyNameB).exists && Bodies.(BodyNameB).contact
                                if  bitand(World.ecoder_x(fix(x_id)),2^bodyB.ord)>0 && bitand(World.ecoder_y(fix(y_id)),2^bodyB.ord)>0 && bitand(World.ecoder_z(fix(z_id-1)),2^bodyB.ord)>0
                                    id=[x_id y_id z_id-1]';
                                    rB=World.box(2,:)'-(id-World.Min)*delta; 
                                    if World.debuging
                                        fprintf('DOWN');
                                    end
                                    
                                    [ identation, value , P, Q] = contact( rA, rB, body, bodyB);
                                    
                                    if identation == 1
                                        World.contact=true;
                                        if value> Bodies.(BodyName).delta
                                            Bodies.(BodyName).delta=value;
                                            f=World.K*value^(3/2);
                                            af= f*bodyB.normal(rAP);
                                            rBP=bodyB.A'*(rB-bodyB.r); 
                                            bf= -f*body.normal(rBP);   
                                            Bodies.(BodyName).Tf=af;
                                            Bodies.(BodyNameB).Tf=bf;
                                            fprintf('A(%f(t=%f) -%s -- %s -> %f (%f,%f,%f)-(%f,%f,%f)\n',identation, Time, BodyName, BodyNameB, value, af(1),af(2),af(3),bf(1),bf(2),bf(3));
                                        end
                                     end
                                    
                                end
                            end
                        end
                    end  
                    
                    % right contact
                    if y_id<id_end(2) && World.ecoder_x(fix(x_id))>0 && World.ecoder_y(fix(y_id+1))>0 && World.ecoder_z(fix(z_id))>0
                        for index=1:indexE-1
                            BodyNameB=BodyList{index};
                            bodyB=Bodies.(BodyNameB);
                            if Bodies.(BodyNameB).exists && Bodies.(BodyNameB).contact
                                if  bitand(World.ecoder_x(fix(x_id)),2^bodyB.ord)>0 && bitand(World.ecoder_y(fix(y_id+1)),2^bodyB.ord)>0 && bitand(World.ecoder_z(fix(z_id)),2^bodyB.ord)>0
                                    id=[x_id y_id+1 z_id]';
                                    rB=World.box(2,:)'-(id-World.Min)*delta; 
                                    if World.debuging
                                        fprintf('UP');
                                    end
                                    [ identation, value, P, Q] = contact( rA, rB, body, bodyB);
                                    if identation == 1
                                        if value> Bodies.(BodyName).delta
                                            Bodies.(BodyName).delta=value;
                                            f=World.K*value^(3/2);
                                            af= f*bodyB.normal(rAP);
                                            rBP=bodyB.A'*(rB-bodyB.r); 
                                            bf= -f*body.normal(rBP);   
                                            Bodies.(BodyName).Tf=af;
                                            Bodies.(BodyNameB).Tf=bf;
                                            fprintf('A(%f(t=%f) -%s -- %s -> %f (%f,%f,%f)-(%f,%f,%f)\n',identation, Time, BodyName, BodyNameB, value, af(1),af(2),af(3),bf(1),bf(2),bf(3));
                                        end                                   
                                    end
                                end
                            end
                        end
                    end 
                     % left contact
                    if y_id>id_ini(2) && World.ecoder_x(fix(x_id))>0 && World.ecoder_y(fix(y_id-1))>0 && World.ecoder_z(fix(z_id))>0
                        for index=1:indexE-1
                            BodyNameB=BodyList{index};
                            bodyB=Bodies.(BodyNameB);
                            if Bodies.(BodyNameB).exists && Bodies.(BodyNameB).contact
                                if  bitand(World.ecoder_x(fix(x_id)),2^bodyB.ord)>0 && bitand(World.ecoder_y(fix(y_id-1)),2^bodyB.ord)>0 && bitand(World.ecoder_z(fix(z_id)),2^bodyB.ord)>0
                                    id=[x_id y_id-1 z_id]';
                                    rB=World.box(2,:)'-(id-World.Min)*delta; 
                                    if World.debuging
                                        fprintf('LEFT');
                                    end
                                    [ identation, value, P, Q ] = contact( rA, rB, body, bodyB);
                                    if identation == 1
                                        World.contact=true;
                                         if value> Bodies.(BodyName).delta
                                            Bodies.(BodyName).delta=value;
                                            f=World.K*value^(3/2);
                                            af= f*bodyB.normal(rAP);
                                            rBP=bodyB.A'*(rB-bodyB.r); 
                                            bf= -f*body.normal(rBP);   
                                            Bodies.(BodyName).Tf=af;
                                            Bodies.(BodyNameB).Tf=bf;
                                            fprintf('A(%f(t=%f) -%s -- %s -> %f (%f,%f,%f)-(%f,%f,%f)\n',identation, Time, BodyName, BodyNameB, value, af(1),af(2),af(3),bf(1),bf(2),bf(3));
                                         end
                                    end
                                end
                            end
                        end
                    end 
                    
                     % right contact
                    if x_id<id_end(1) && World.ecoder_x(fix(x_id+1))>0 && World.ecoder_y(fix(y_id))>0 && World.ecoder_z(fix(z_id))>0
                        for index=1:indexE-1
                            BodyNameB=BodyList{index};
                            bodyB=Bodies.(BodyNameB);
                            if Bodies.(BodyNameB).exists && Bodies.(BodyNameB).contact
                                if  bitand(World.ecoder_x(fix(x_id+1)),2^bodyB.ord)>0 && bitand(World.ecoder_y(fix(y_id)),2^bodyB.ord)>0 && bitand(World.ecoder_z(fix(z_id)),2^bodyB.ord)>0
                                    id=[x_id+1 y_id z_id]';
                                    rB=World.box(2,:)'-(id-World.Min)*delta; 
                                    if World.debuging
                                        fprintf('RIGTH2');
                                    end

                                    [ identation, value, P, Q ] = contact( rA, rB, body, bodyB);
                                    if identation == 1
                                        World.contact=true;
                                         if value> Bodies.(BodyName).delta
                                            Bodies.(BodyName).delta=value;
                                            f=World.K*value^(3/2);
                                            af= f*bodyB.normal(rAP);
                                            rBP=bodyB.A'*(rB-bodyB.r); 
                                            bf= -f*body.normal(rBP);   
                                            Bodies.(BodyName).Tf=af;
                                            Bodies.(BodyNameB).Tf=bf;
                                            fprintf('A(%f(t=%f) -%s -- %s -> %f (%f,%f,%f)-(%f,%f,%f)\n',identation, Time, BodyName, BodyNameB, value, af(1),af(2),af(3),bf(1),bf(2),bf(3));
                                        end
                                    end
                                end
                            end
                        end
                    end 
                    
                     % left contact
                    if y_id>id_ini(1) && World.ecoder_x(fix(x_id-1))>0 && World.ecoder_y(fix(y_id))>0 && World.ecoder_z(fix(z_id))>0
                        for index=1:indexE-1
                            BodyNameB=BodyList{index};
                            bodyB=Bodies.(BodyNameB);
                            if Bodies.(BodyNameB).exists && Bodies.(BodyNameB).contact
                                if  bitand(World.ecoder_x(fix(x_id-1)),2^bodyB.ord)>0 && bitand(World.ecoder_y(fix(y_id)),2^bodyB.ord)>0 && bitand(World.ecoder_z(fix(z_id)),2^bodyB.ord)>0
                                    id=[x_id-1 y_id z_id]';
                                    rB=World.box(2,:)'-(id-World.Min)*delta; 
                                    if World.debuging
                                        fprintf('LEFT');
                                    end
                                    [ identation, value, P, Q ] = contact( rA, rB, body, bodyB);
                                    if identation == 1
                                        World.contact=true;
                                        if value> Bodies.(BodyName).delta
                                            Bodies.(BodyName).delta=value;
                                            f=World.K*value^(3/2);
                                            af= f*bodyB.normal(rAP);
                                            rBP=bodyB.A'*(rB-bodyB.r); 
                                            bf= -f*body.normal(rBP);   
                                            Bodies.(BodyName).Tf=af;
                                            Bodies.(BodyNameB).Tf=bf;
                                            fprintf('A(%f(t=%f) -%s -- %s -> %f (%f,%f,%f)-(%f,%f,%f)\n',identation, Time, BodyName, BodyNameB, value, af(1),af(2),af(3),bf(1),bf(2),bf(3));
                                        end
                                    end
                                end
                            end
                        end
                    end 
               
                    World.ecoder_x(fix(x_id))= bitor(World.ecoder_x(fix(x_id)),2^body.ord);
                    World.ecoder_y(fix(y_id))= bitor(World.ecoder_y(fix(y_id)),2^body.ord);
                    World.ecoder_z(fix(z_id))= bitor(World.ecoder_z(fix(z_id)),2^body.ord);       
                    
                    x= fix(x_id/2);
                    while x>0 && bitand(World.ecoder_x(x),2^body.ord)== 0
                        World.ecoder_x(x)= bitor(World.ecoder_x(x),2^body.ord);
                        x= fix(x/2);
                    end

                    y= fix(y_id/2);
                    while y>0 && bitand(World.ecoder_y(y),2^body.ord)== 0
                        World.ecoder_y(y)= bitor(World.ecoder_y(y),2^body.ord);
                        y= fix(y/2);
                    end

                    z= fix(z_id/2);
                    while z>0 && bitand(World.ecoder_z(z),2^body.ord)== 0
                        World.ecoder_z(z)= bitor(World.ecoder_z(z),2^body.ord);
                        z= fix(z/2);
                    end
                    
                end % in the body
                
                
            end % end body box z
       
        end % end body box y
        
    end % end body box x
end

