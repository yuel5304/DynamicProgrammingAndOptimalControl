function [ J_opt, u_opt_ind ] = ValueIteration(P, G)
%VALUEITERATION Value iteration
%   Solve a stochastic shortest path problem by Value Iteration.
%
%   [J_opt, u_opt_ind] = ValueIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%
%       J_opt:
%       	A (K x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (K x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).
global K L HOVER

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration ? we shouldn't take the terminal state into value iteration
% consider.
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)
tmp = zeros(K);
J = tmp(:,1);
J_opt = J;
convergence = false;
J_new = J;
nonTERMINAL = [1:TERMINAL_STATE_INDEX-1 TERMINAL_STATE_INDEX+1:K];
err_tol = 1e-3;
while ~convergence
    for i = 1:K
        if i ~= TERMINAL_STATE_INDEX
            min = G(i,1) + P(i,nonTERMINAL,1)*J(nonTERMINAL);
            for u = 2:L
                tmp = G(i,u) + P(i,nonTERMINAL,u)*J(nonTERMINAL);
                if min > tmp
                    min = tmp;
                  
                end
            end
            J_new(i) = min;
        end
    end
    J = J_new;
    if sum(abs(J_opt-J)) <= err_tol
        
        convergence = true;
    end
    J_opt = J;
end
%get the convergent cost value
J_opt = J;
%calculate out the optimal input out of the convergent optimal cost.
tmp = zeros(K);
u_opt_ind = tmp(:,1);
for i = 1:K
    if i ~= TERMINAL_STATE_INDEX
        min = G(i,1) + P(i,:,1)*J_opt;
        for u = 2:L
            tmp = G(i,u) + P(i,:,u)*J;
            if min > tmp
                min = tmp;
                u_opt_ind(i) = u;
            end
        end
    end
end
        


end