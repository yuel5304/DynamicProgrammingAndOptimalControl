function P = shoot(x,y,map)
%   probability being shooted by angry resident
%   Input arguments:
%
%       x,y:
%           Two integer indicating the 2 dimensions of location.
%
%       map:
%           A (M x N)-matrix describing the world. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%       P: 
%           A (1 x 2)-vector describing the probability of being hitted and not.

% locate the number and position of shooter in the neighborhood(diameter R=2)

    global SHOOTER R GAMMA
    global row col
    p = [];
    for i=-R:R
        if x+i < 1 || x+i > row
            continue
        end
        for j=-R:R
            if y+j < 1 || y+j > col
                continue
            end
            if map(x+i,y+j) == SHOOTER
                d = abs(i)+abs(j);
                if d<=R
                    p=[p,GAMMA/(d+1)];
                end
            end
        end
    end
    P = 1;
    for i=1:size(p)
        P = P*(1-p(i));
    end
    P = [1-P,P];
end
