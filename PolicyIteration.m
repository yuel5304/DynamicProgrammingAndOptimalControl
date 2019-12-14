function [ J_opt, u_opt_ind ] = PolicyIteration( P, G )
%POLICYITERATION Policy iteration
%   Solve a stochastic shortest path problem by Policy Iteration.
%
%   [J_opt, u_opt_ind] = PolicyIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%
%       P:
%           A (k x k x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (k x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%
%       J_opt:
%       	A (k x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (k x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).

global K HOVER L 
global NORTH SOUTH EAST WEST HOVER 

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)
J = zeros(K,1);
J_opt = J;
u_opt_ind = ones(K,1);
u_new = u_opt_ind;
convergence = false;
err_tol = 1e-6;
nonTERMINAL = [1:TERMINAL_STATE_INDEX-1 TERMINAL_STATE_INDEX+1:K];
%initialize the valid input for each state
for i = 1:K
    utmp = [];
    for input = [NORTH SOUTH EAST WEST HOVER]
        if G(i,input)~= inf
            u_opt_ind(i) = input;
            utmp = [utmp,input];
        end
    end
    u{i} = utmp;
end
while ~convergence
    for i = 1:K
        if i ~= TERMINAL_STATE_INDEX
            J(i) = G(i,u_opt_ind(i)) + P(i,nonTERMINAL,u_opt_ind(i))*J_opt(nonTERMINAL);%the cost assoicated with current input series.
        end
    end
    for i = 1:K
        input = u{i};
        Pi = reshape(P(i,:,input),K,length(input));
        Gi = reshape(G(i,input),length(input),1);
        tmp = Gi+Pi'*J;
        [min_value, index] = min(tmp,[],1);
        u_opt_ind(i) = input(index);
    end
        
    
    if max(abs(J_opt-J)) <= err_tol
        convergence = true;
    end
    J_opt = J;
end


end
