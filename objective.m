% Computes the distance between a goal position
% and 'current end effector' position.
%
% goalPosition = cartesian [x y z]
% qState = joint angles in rad
% qSym = subs parameters symbolic names of joints angles
% T = the tranformation matrix from base to 'current end efector'
function [ distance ] = objective( goalPosition, qState, qSym, T0j )

    p = subs(T0j, qSym, qState);
    p = p(1:3,4);

    distance = (goalPosition-p)'*(goalPosition-p);

end

