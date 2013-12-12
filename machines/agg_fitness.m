function [final, paths ] = agg_fitness( file_list )
%AGG_FITNESS Aggregates the data in all the fitness logs
% [final, paths] = agg_fitness(file_list)

final = [];
paths = [];
for i=1:length(file_list),
    fname = file_list{i};
    disp(fname);
    if isempty(fname)
        break;
    end
    fit_info = read_fitness(fname);
    last_col = fit_info(:, end);
    %n = length(last_col);
    a = max(fit_info);
     %   m = length(a);
    final = vertcat2(final, last_col);
    paths = extendcat(paths, a');
    %avg(1:m, i) = a';
end


end



