function [ gen_data ] = read_fitness( filename )

input = fopen(filename);

gen_data = [];

line = fgetl(input);
while line ~= -1
    s = strsplit(line, ':');
    vals = strsplit(s{2}, ' ');
    col_vals = str2double(vals)';
    %col_vals = col_vals(~isnan(col_vals));
    gen_data = [gen_data, col_vals];
    line = fgetl(input);
end
fclose(input);
end

