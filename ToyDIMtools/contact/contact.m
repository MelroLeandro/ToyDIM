function [ identation, value, P, Q] = contact( P, Q, c1, c2)

global World

% Error: grid to big
if c2.shape(c2.A'*(P-c2.r)) || c1.shape(c1.A'*(Q-c1.r))
    value=0;
    if c2.shape(c2.A'*(P-c2.r)) && c1.shape(c1.A'*(Q-c1.r))
        identation=2;
    else if c2.shape(c2.A'*(P-c2.r))
            identation=3;
        else if c1.shape(c1.A'*(Q-c1.r))
                identation=4;
            end
        end
    end        
else
    error = World.Error;

    new=(P+Q)/2;
    P_M=new;
    Q_M=new;

    if World.debuging 
        fprintf('P in C1 %f\n', c1.shape(c1.A'*(P-c1.r)));
        fprintf('P_M in C1 %f\n', c1.shape(c1.A'*(P_M-c1.r)));
        fprintf('Q in C1 %f\n', c1.shape(c1.A'*(Q-c1.r)));
        fprintf('Q_M in C1 %f\n', c1.shape(c1.A'*(Q_M-c1.r)));
        fprintf('----------\n');

        fprintf('0');
    end

    while c1.shape(c1.A'*(P_M-c1.r))
        P_M=(P_M+Q)/2;
    end

    if World.debuging 
        fprintf('1');

        fprintf('P in C2 %f\n', c2.shape(c2.A'*(P-c2.r)));
        fprintf('P_M in C2 %f\n', c2.shape(c2.A'*(P_M-c2.r)));
        fprintf('Q in C2 %f\n', c2.shape(c2.A'*(Q-c2.r)));
        fprintf('Q_M in C2 %f\n', c2.shape(c2.A'*(Q_M-c2.r)));
        fprintf('----------\n');
    end    

    while c2.shape(c2.A'*(Q_M-c2.r))
        Q_M=(P+Q_M)/2;
    end

    if World.debuging 
        fprintf('2');
    end

    if c1.shape(c1.A'*(Q_M-c1.r))

        identation = 1;
        while norm(P-P_M) > error
            P_new = (P+P_M)/2;
            if c2.shape(c2.A'*(P_new-c2.r))
                P_M = P_new;
            else
                P = P_new;
            end
        end

        if World.debuging
            fprintf('3');
        end

        while norm(Q-Q_M) > error
            Q_new = (Q+Q_M)/2;
            if c1.shape(c1.A'*(Q_new-c1.r))
                Q_M = Q_new;
            else
                Q = Q_new;
            end
        end
        if World.debuging
            fprintf('4');
        end
    else

        identation = 0;
        while norm(P-P_M) > error
            P_new = (P+P_M)/2;
            if c1.shape(c1.A'*(P_new-c1.r))
                P = P_new;
            else
                P_M = P_new;
            end
        end

        if World.debuging
            fprintf('5');
        end

        while norm(Q-Q_M) > error
            Q_new = (Q+Q_M)/2;
            if c2.shape(c2.A'*(Q_new-c2.r))
                Q = Q_new;
            else
                Q_M = Q_new;
            end
        end  
        if World.debuging
            fprintf('6');
        end
    end

    value = norm(P-Q);
end

end

