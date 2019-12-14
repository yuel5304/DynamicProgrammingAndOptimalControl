function [P,G] = loop(new,P,G,i,u,p_wind,valid,varargin)
%LOOP exhuastically search all over the dynamics space: 
%   given input u at state i
%   calculate the new location  = new
%   if wind, wiht probability p_wind = P_WIND, location = tmp_new = new + wind
%       if crash, return to the "BASE without package", cost = Nc+1
%       else, 
%           if hitted, with probability p=shooter(location,param)*p_wind
%              return to the "BASE without package", cost = Nc+1
%           else
%              safe
%   else, with probability p_wind = 1-P_WIND, location = new
%       {cannot crash without wind, because it will be invalid input in the first place}
%       if hitted, with probability p_shoot=shooter(location,param)*p_wind
%          return to the "BASE without package", cost = Nc+1
%       else
%          safe
%   if safe, cost = 1
%       if at "PICK_UP without package"
%           "PICK_UP with package"
%       else
%           safely stay at location, and the state of package stays the same
%                with prob. =  P_WIND*(1-p_shoot), location = tmp_new 
%                with prob. = (1-P_WIND)*(1-p_shoot), location = new
%
%
%   Input arguments:
%
%       new:
%           A (1 x 2)-vector indicating the new location after either input or dynamics.
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied. The
%           first layer is the current state, the second is the next state.
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the expected stage cost if we are in state i and 
%           apply control input l.
%       i:
%           An integer describing the current state index.
%       u:
%           An integer describing the current input.
%       p_wind: 
%           A double describing the probability of wind in current
%           situation. p_wind = {P_WIND, 1-P_WIND}.
%       valid:
%           A boolean describing the vadility status of current location.
%           Validity means whether out of boudary or tree. 
%       varargin:
%           length-changable parameters. includes, 
%           map_idnex: An integer indicating the state index of given location and state of package.
%           map: A (M x N)-matrix describing the world.
%          
%   Output arguments:
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied. The
%           first layer is the current state, the second is the next state.
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the expected stage cost if we are in state i and 
%           apply control input l.
    global PICK_UP 
    global STARTING_STATE_INDEX 
    global Nc

    %% tree or boundary
    if ~valid
        %if crash, return to the starting state
        P(i, STARTING_STATE_INDEX,u) = P(i, STARTING_STATE_INDEX,u) + p_wind;
        P(i+1, STARTING_STATE_INDEX,u) = P(i+1, STARTING_STATE_INDEX,u) + p_wind;%no matter what state you are, start over again.
        G(i,u) = G(i,u) + p_wind*(Nc);
        G(i+1,u) = G(i+1,u) + p_wind*(Nc);
        return

    end


    %% shooter
    map_index = varargin{1};
    map = varargin{2};
    state_index = map_index*2-1;  %new state_index after wind and input for phi=0. for phi=1, state_index+1


    p_shooter = shoot(new(1),new(2),map); %[p_hit, p_not_hit]
    %if hitted, return to the starting state
    P(i, STARTING_STATE_INDEX,u) = P(i, STARTING_STATE_INDEX,u) + p_wind*p_shooter(1);
    P(i+1, STARTING_STATE_INDEX,u) = P(i+1, STARTING_STATE_INDEX,u) + p_wind*p_shooter(1); %no matter what state, if crash, start over again.
    G(i,u) = G(i,u) + p_wind*p_shooter(1)*(Nc);
    G(i+1,u) = G(i+1,u) + p_wind*p_shooter(1)*(Nc);
    %% determine the action if not hitted
    if map(new(1),new(2))== PICK_UP
        %%if arriving at pick up station without package and then pick up the
        %%packages( same location )
        P(i,state_index+1,u) = P(i,state_index+1,u) + p_wind*p_shooter(2);
        
    else
        P(i, state_index, u) = P(i, state_index, u) + p_wind*p_shooter(2);
    end
    P(i+1, state_index+1, u) = P(i+1, state_index+1, u) + p_wind*p_shooter(2); %no change when with package or normal state.
    G(i,u) = G(i,u) + 1*p_wind*p_shooter(2);
    G(i+1,u) = G(i+1,u) + 1*p_wind*p_shooter(2);
    return
end

