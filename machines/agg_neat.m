function [ best, mean_dist, n_species ] = agg_neat( file_list )
%AGG_NEAT Aggregates the data in all the fitness logs in file_list
%   Detailed explanation goes here
best = [];
mean_dist = [];
n_species = [];

for i=1:length(file_list),
    fname = file_list{i};
    disp(fname);
    if isempty(fname)
        break;
    end
    [run_best, ~, run_dist] = read_neat(fname); % don't need to save the species size or avg info
    
    % the number of species present
    not_empty = ~isnan(run_dist);
    species_count = (sum(not_empty))';
    n_species = vertcat2(n_species, species_count);
    
    % then, the best of the species present in the last generation
   
    best = vertcat2(best, max(run_best)');
    
    %the mean distance... don't know how to do this except columnwise
    mean_dist = vertcat2(mean_dist, nanmean(run_dist)');
    
end

end

