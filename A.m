function M = A(p)
    e0 = p(1); 
    e1 = p(2); 
    e2 = p(3); 
    e3 = p(4);
    M = [ e0*e0 + e1*e1 - 0.5,e1*e2 - e0*e3, e1*e3 + e0*e2
           e1*e2 + e0*e3,e0*e0 + e2*e2 - 0.5, e2*e3 - e0*e1
           e1*e3 - e0*e2, e2*e3 + e0*e1, e0*e0 + e3*e3 - 0.5];
    M=2*M;
end

