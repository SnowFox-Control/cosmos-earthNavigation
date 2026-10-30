function result=rocket_two_burn_sa(varargin)
% Multi-start simulated annealing with Metropolis acceptance.
result=rocket_two_burn_search('SA',varargin{:});
end
