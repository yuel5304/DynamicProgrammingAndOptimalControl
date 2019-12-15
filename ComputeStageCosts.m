function G = ComputeStageCosts( stateSpace, map, map_index)
%COMPUTESTAGECOSTS Compute stage costs.
% 	Compute the stage costs for all states in the state space for all
%   control inputs.
%
%   G = ComputeStageCosts(stateSpace, map) 
%   computes the stage costs for all states in the state space for all
%   control inputs.
%
%   Input arguments:
%
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the world. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%       map_index:
%           A (M x N)-matrix describing mapping relation between position in map
%           and index i without package in stateSpace, thus the relevant
%           index with package in stateSpace is i+1.
%
%
%   Output arguments:
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the expected stage cost if we are in state i and 
%           apply control input l.

    %global GAMMA R P_WIND Nc
    %global FREE TREE SHOOTER PICK_UP DROP_OFF BASE

    global P_WIND Nc
    global NORTH SOUTH EAST WEST HOVER 
    global K TERMINAL_STATE_INDEX 

    input = [NORTH SOUTH EAST WEST HOVER];
    L = length(input);
    direction = [[0,+1];[0,-1];[1,0];[-1,0];[0,0]];
    P = zeros(K,K,L);
    G = zeros(K,L);
    for i = 1:2:K %loop over position
       
        pos = [stateSpace(i,1),stateSpace(i,2)];
        %given on this current position no matter with or without package
        for u = input %given on this current input
            % input
            direction(u,:);
            new = pos + direction(u,:);
            % without wind
            flag = validate(new,map);
            if ~flag
                G(i,u) = inf; %penalize invalid input
                G(i+1,u) = inf;
                P(i,u) = 0;
                continue
            end
            p_wind = 1- P_WIND;%prob of no wind, therefore stay.
            [P,G] = loop(new,P,G,i,u,p_wind,true,map_index(new(1),new(2)),map);
            % with wind
            for x=[NORTH SOUTH EAST WEST]
                p_wind = P_WIND*0.25;%sharing equal probability to be moved in 4 direction.
                tmp_new = new + direction(x,:);
                flag = validate(tmp_new,map);
                if flag == false
                    [P,G] = loop(tmp_new,P,G,i,u,p_wind,flag);
                else 
                    [P,G] = loop(tmp_new,P,G,i,u,p_wind,flag,map_index(tmp_new(1),tmp_new(2)),map);
                end
            end    
        end
        if  i+1 == TERMINAL_STATE_INDEX %i is the index for without package
           P(i+1,1:K,input) = 0;
           P(i+1,i+1,input) = 1;
           G(i+1,input) = 0; %no cost at terminal state
        end
    end
end
