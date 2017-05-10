function  regist(v1,v2,A,Q)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    global World
    [l1,c1]=size(v1);
    [l2,c2]=size(v2);
    [la,ca]=size(A);
    [lq,cq]=size(Q);
    L=[];
    for i=1:l1
        L=[L, double(v1(i,:))];
    end
    for i=1:l2
        L=[L, double(v2(i,:))];
    end
    for i=1:la
        L=[L, double(A(i,:))];
    end
    for i=1:lq
        L=[L, double(Q(i,:))];
    end
    World.V=[World.V; L];
end

