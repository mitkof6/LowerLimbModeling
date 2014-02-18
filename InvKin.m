function [ q ,fval ] = InvKin( pos, qprev, T0j, qSym)

    DOF = length(pos(:, 1));
    options = optimset('Algorithm','interior-point','Display','off') ;    
    A=[];b=[];
    lb = [-Inf,-pi/2,pi/18,-pi/2,-pi,0];
    ub = [Inf,pi/2,3*pi/4,pi/2,pi/18,3*pi/4];
    
    q = zeros(DOF,1);
    
    for i = 1:DOF
        
        T = T0j(:,:,i);
        T = subs(T, qSym(1:i-1), q(1:i-1));
        qSym_i = qSym(i);
        X0 = qprev(i);
                
        [q_i, fval] = fmincon(@(q_i) objective(pos(i,:)', q_i, qSym_i, T), ...
            X0, A, b, [], [], lb(i), ub(i), [], options) ;
        q(i) = q_i;
    end
end

