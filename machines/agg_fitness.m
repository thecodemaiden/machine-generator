function [final, avg ] = agg_fitness( file_list )

final = [];
avg = [];
%AGG_FITNESS Aggregates the data in all the fitness logs in file_list
for i=1:length(file_list),
    fname = file_list{i};
    disp(fname);
    if isempty(fname)
        break;
    end
    fit_info = read_fitness(fname);
    last_col = fit_info(:, end);
    n = length(last_col);
    a = mean(fit_info);
        m = length(a);
    final(1:n, i) = last_col;
    avg(1:m, i) = a';
end


end

