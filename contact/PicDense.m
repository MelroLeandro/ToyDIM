function  PicDense(body)
    
    global World

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
                
                if body.exists && body.shape(rAP)> World.MinDense 
                                                
               
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

