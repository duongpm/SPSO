%
% Create random paths (solutions)
% 

function sol=CreateRandomSolution(VarSize,VarMin,VarMax) 
    % Random path nodes
    sol.r=unifrnd(VarMin.r,VarMax.r,VarSize);
    sol.psi=unifrnd(VarMin.psi,VarMax.psi,VarSize);
    sol.phi=unifrnd(VarMin.phi,VarMax.phi,VarSize);
end