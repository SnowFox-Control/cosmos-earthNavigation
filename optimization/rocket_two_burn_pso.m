function result=rocket_two_burn_pso(varargin)
% Pure PSO entry point; no SQP or other local refinement.
result=rocket_two_burn_search('PSO',varargin{:});
end
