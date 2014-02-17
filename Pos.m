function [ pos ] = Pos( T0j, q, qz )

    pos = [];
    for i = 1:length(T0j(1, 1, :))
        pos = cat(1, pos, subs(T0j(1:3, 4, i)', q, qz));
        
    end
    pos=vpa(pos);

end

