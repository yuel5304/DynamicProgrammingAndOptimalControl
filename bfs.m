function u = bfs(P)
%BFS finds one proper policy  

global K L TERMINAL_STATE_INDEX STARTING_STATE_INDEX 
global NORTH SOUTH EAST WEST HOVER 


% find proper initialization
idx_reachable = [TERMINAL_STATE_INDEX];
u = zeros(K,1);
u(TERMINAL_STATE_INDEX) = HOVER;

unvisited = (zeros(K,1)==0);
unvisited(TERMINAL_STATE_INDEX) = false;
tmp = 1:K;
tmp(TERMINAL_STATE_INDEX) = [];
while(any(u==0))
     
        P_temp = squeeze(sum(P(unvisited,idx_reachable,:),2));
        [value,col] = max(P_temp,[],2);%mark the possible previous state that can lead to "reachable set"
        C = find(value~=0);
        u(tmp(C)) = col(C);
  
        idx_reachable = [idx_reachable,tmp(C)];
        unvisited(tmp(C)) = false;
        tmp(C) = [];
        
end


u(TERMINAL_STATE_INDEX) = HOVER;
    