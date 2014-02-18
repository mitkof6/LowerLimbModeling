% For given joint positions in cartesian space
% generates joints angles recursivly from base
% to end effector
%
% q = joints angles
% fval = error
% positions = DOFx3 vector of joints positions
% qPrevius = previus joints configurations
% T0j = tranformations from base to joint j
% qSym = joints symbolics for subs
function [ q ,fval ] = ikine( postions, qPrevius, T0j, qSym)

    DOF = length(postions(:, 1));
    
    options = optimset('Algorithm','interior-point','Display','off') ;
    
    Aine = [];
    bine = [];
    
    lb = [-Inf,-pi/2,pi/18,-pi/2,-pi,0];
    ub = [Inf,pi/2,3*pi/4,pi/2,pi/18,3*pi/4];
    
    q = zeros(DOF,1);
  
    for i = 1:DOF
        
        T = T0j(:,:,i);
        T = subs(T, qSym(1:i-1), q(1:i-1));
        qSym_i = qSym(i);
        X0 = qPrevius(i);
                
        [q_i, fval] = fmincon(@(q_i) objective(postions(i,:)', q_i, qSym_i, T), ...
            X0, Aine, bine, [], [], lb(i), ub(i), [], options) ;
        
        q(i) = q_i;
    end
end

