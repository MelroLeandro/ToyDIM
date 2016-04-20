function BOXintergartion(x_id,y_id,z_id,r,corpoA,rAP,BodyNameA,indexEA,delta)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    global World % used to describe the world
    global Bodies

    
    % bodies in the box
    
    for indexE=1:indexEA-1
        BodyNameB=World.BodyList{indexE};

        corpoB=Bodies.(BodyNameB);
        
        
        if  bitand(World.ecoder_x(x_id),2^corpoB.ord)>0 && bitand(World.ecoder_y(y_id),2^corpoB.ord)>0 && bitand(World.ecoder_z(z_id),2^corpoB.ord)>0
            
            fprintf('%s --> %s\n',BodyNameA,BodyNameB); 
            
            x  = r;
            deltaA = delta/2;
            
            while deltaA > World.Error
                deltaA = deltaA/2;

                xA=x+deltaA*[1 0 0]';
               
                rAP=corpoA.A'*(xA-corpoA.r);
                rBP=corpoB.A'*(xA-corpoB.r);

                if corpoA.shape(rAP) && corpoB.shape(rBP)
                    fprintf('xAa-> %f %f %f - %f\n',xA(1),xA(2),xA(3),rAP'*rAP);
                    x=xA;
                end
                
                xB=x-deltaA*[1 0 0]';

                rAP=corpoA.A'*(xB-corpoA.r);
                rBP=corpoB.A'*(xB-corpoB.r);

                if corpoA.shape(rAP) && corpoB.shape(rBP)
                    fprintf('xBe-> %f %f %f - %f\n',xA(1),xA(2),xA(3),rAP'*rAP);
                    x=xB;
                end
                
                y1 = x+deltaA*[0 1 0]';
                
                rAP=corpoA.A'*(y1-corpoA.r);
                rBP=corpoB.A'*(y1-corpoB.r);
                
                 if corpoA.shape(rAP) && corpoB.shape(rBP)
                     fprintf('yBe\n');
                     x=y1;
                 end
                 

                y1=x+deltaA*[0 1 0]';

                rAP=corpoA.A'*(y1-corpoA.r);
                rBP=corpoB.A'*(y1-corpoB.r);

                if corpoA.shape(rAP) && corpoB.shape(rBP)
                    fprintf('yAa-> %f %f %f - %f\n',y1(1),y1(2),y1(3),rAP'*rAP);
                    x=y1;
                end

                y2=x-deltaA*[0 1 0]';
                                
                rAP=corpoA.A'*(y2-corpoA.r);
                rBP=corpoB.A'*(y2-corpoB.r);

                if corpoA.shape(rAP) && corpoB.shape(rBP)
                    fprintf('yBe-> %f %f %f - %f\n',y2(1),y2(2),y2(3),rAP'*rAP);
                    x=y2;
                end

                z1=x+deltaA*[0 1 0]';

                rAP=corpoA.A'*(z1-corpoA.r);
                rBP=corpoB.A'*(z1-corpoB.r);

                if corpoA.shape(rAP) && corpoB.shape(rBP)
                    fprintf('zAa-> %f %f %f - %f\n',z1(1),z1(2),z1(3),rAP'*rAP);
                    x=z1;
                end
                
                z2=x-deltaA*[0 1 0]';

                rAP=corpoA.A'*(z2-corpoA.r);
                rBP=corpoB.A'*(z2-corpoB.r);

                if corpoA.shape(rAP) && corpoB.shape(rBP)
                    fprintf('zBe-> %f %f %f - %f\n',z2(1),z2(2),z2(3),rAP'*rAP);
                    x=z2;
                end
            end
            
        end % if
    end
     
end

