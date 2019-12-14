function flag = validate(new,map)
%VALIDATE: validate whether current input is valid or not
%   Input arguments:
%
%       new:
%           A (1 x 2)-vector indicating the location.
%
%       map:
%           A (M x N)-matrix describing the world. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%
%       flag:
%           A boolean describing the validity.

    global row col TREE


    if (new(1)>row || new(2)>col || new(1)<1 || new(2)<1) || map(new(1),new(2)) == TREE 
        flag = false;

    else
        flag = true;
    end
    return
end

