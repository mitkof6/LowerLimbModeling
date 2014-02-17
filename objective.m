function [ dist ] = objective( pos, q, qSym, T )

    p = subs(T, qSym, q);
    p=p(1:3,4);

    dist = (pos-p)'*(pos-p);

end

