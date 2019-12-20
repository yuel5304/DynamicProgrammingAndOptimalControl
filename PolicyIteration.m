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
J = ones(K,1);
J_opt = J;
J_opt(TERMINAL_STATE_INDEX) = 0;
u_opt_ind = ones(K,1)*HOVER;

convergence = false;
err_tol = 1e-5;
nonTERMINAL = [1:TERMINAL_STATE_INDEX-1 TERMINAL_STATE_INDEX+1:K];

u_opt_ind = bfs(P);
while ~convergence
    Pi = eye(K,K);
    Gi = eye(K,1);
    for i=1:K
        if i ~= TERMINAL_STATE_INDEX
            Pi(i,:) = squeeze(P(i,:,u_opt_ind(i)));
            Gi(i) = G(i,u_opt_ind(i));
        end
    end
  
    %get the converged cost for current policy
    %det(eye(K-1,K-1)-Pi(nonTERMINAL,nonTERMINAL))
    J(nonTERMINAL) = inv(eye(K-1,K-1)-Pi(nonTERMINAL,nonTERMINAL))*Gi(nonTERMINAL);
    %update current policy
    for i = 1:K
          if i ~= TERMINAL_STATE_INDEX 
            
            Pi = squeeze(P(i,:,:));
            Gi = G(i,:);    
            tmp = Gi+J'*Pi;         
            [min_value, u_opt_ind(i)] = min(tmp);
      
          end              
    end
    
    if max(max((abs(J-J_opt)))) <= err_tol
        convergence = true;
    end
    J_opt = J;
    
    
end
J_opt(TERMINAL_STATE_INDEX) = 0;
end

