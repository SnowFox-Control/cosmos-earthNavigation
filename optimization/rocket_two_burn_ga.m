function result=rocket_two_burn_ga(varargin)
% Compatibility alias: the active optimizer is now PSO.
warning('rocket:RenamedOptimizer','Use rocket_two_burn_pso; this alias now runs PSO.');
result=rocket_two_burn_pso(varargin{:});
end
