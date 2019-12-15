function stateIndex = ComputeTerminalStateIndex(stateSpace, map)
%ComputeTerminalStateIndex Compute the index of the terminal state and starting state in the
%stateSpace matrix
%
%   stateIndex = ComputeTerminalStateIndex(stateSpace, map)
%   Computes the index of the terminal state in the stateSpace matrix
%   Input arguments:
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the terrain of the estate map. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%
%       stateIndex (1 X 2): [TERMINAL_STATE_INDEX, STARTING_STATE_INDEX]
%           An integer that is the index of the terminal state and starting state in the
%           stateSpace matrix

    global DROP_OFF BASE 

    tmp = size(stateSpace);
    state = tmp(1,1);
    flag = 0;
    stateIndex = ones(1,2)*(-1);
    for i = 1:state
        if map(stateSpace(i,1),stateSpace(i,2)) == DROP_OFF && stateSpace(i,3) == 1
           stateIndex(1) = i;
           flag = flag + 1;
        end
        if map(stateSpace(i,1),stateSpace(i,2)) == BASE && stateSpace(i,3) == 0
            stateIndex(2) = i;
            flag = flag + 1;
        end
        if flag == 2
            return
        end
    end
end
