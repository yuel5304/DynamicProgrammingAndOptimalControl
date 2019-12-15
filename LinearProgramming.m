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

global K HOVER

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration ?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)

%% LP
%K = size(P,1);
L = size(P,3);
f = -1 * ones(1,K);
A = zeros(K*L,K);
B = zeros(1, K*L);
%J_opt = zeros(K,1);
u_opt_ind = zeros(K,1);
for l=1:L
    A((l-1)*K+1:l*K,:) = eye(K,K)-P(:,:,l);
 
    % make constraint for terminal state free
    a = A((l-1)*K+1:l*K,:);
    a(TERMINAL_STATE_INDEX,:) = 0;
    A((l-1)*K+1:l*K,:) = a;
    B((l-1)*K+1:l*K) = G(:,l);
    
    % make constraint for terminal state free
    b = B((l-1)*K+1:l*K);
    b(TERMINAL_STATE_INDEX) = 0;
    B((l-1)*K+1:l*K) = b; 
end
is_Inf = find(B==Inf);
A(is_Inf,:) = 0;
B(is_Inf) = 0;
J_opt = linprog(f,A,B);
%size(J_opt)
%size(squeeze(P1(1,:,:)))
for k = 1:K
    [~,u_opt_ind(k)] = min ( G(k,:) + J_opt' * squeeze(P(k,:,:)));
end

end

