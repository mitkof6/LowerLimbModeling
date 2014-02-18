% Generates positions based on T0j's
% 
% positions = vector of [x y z] positions
% T0j = transformation from base to joint j
% qSym = joints symbolics for subs
% q = joints angles
function [ positions ] = generatePositions( T0j, qSym, q )

    positions = [];
    for i = 1:length(T0j(1, 1, :))
        positions = cat(1, positions, subs(T0j(1:3, 4, i)', qSym, q));
        
    end
    positions = vpa(positions);

end

