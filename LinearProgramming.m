function [ J_opt, u_opt_ind ] = LinearProgramming(P, G)
%LINEARPROGRAMMING Linear Programming
%   Solve a stochastic shortest path problem by Linear Programming.
%
%   [J_opt, u_opt_ind] = LinearProgramming(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
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

global K HOVER L

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration ?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)

%% LP
length = K-1;
f = -1 * ones(length,1);
A = zeros(length*L,length);
B = zeros(length*L,1);
u_opt_ind = ones(K,1)*HOVER;
nonTERMINAL = [1:TERMINAL_STATE_INDEX-1 TERMINAL_STATE_INDEX+1:K];

for i = 1:K
    if i ~= TERMINAL_STATE_INDEX
        for l=1:L
            A((l-1)*length+1:l*length,:) = eye(length,length)-P(nonTERMINAL,nonTERMINAL,l);
            B((l-1)*length+1:l*length) = G(nonTERMINAL,l);
        end
    end

end
is_Inf = find(B==Inf);
A(is_Inf,:)=0;
B(is_Inf)=0;
J = linprog(f,A,B);
J_opt = zeros(K,1);
J_opt(nonTERMINAL) = J;
for k = 1:K
    if k ~= TERMINAL_STATE_INDEX
        [~,u_opt_ind(k)] = min ( G(k,:) + J_opt' * squeeze(P(k,:,:)),[],2);
    end
end

end

