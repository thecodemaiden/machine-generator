function [ best, avg, distances, species ] = read_neat( filename )
%READ_NEAT Read and process NEAT algorithm
%   Returns the best fitness across all species, mean fitness, cumulative
%   species count (for graph), and distance of species from starting point

input = fopen(filename);

best = [];
avg = [];
distances_raw = [];
species_raw = [];

gen = 1;
line = fgetl(input);
gen_data = [];
while ischar(line)
    if isempty(line)        
        % data is species #, species size, best, worst, mean, std dev, distance
        data_shape = size(gen_data);
        data_len = data_shape(1);
        best = vertcat2(best, gen_data(:,3));
        avg = vertcat2(avg, gen_data(:,4));
        
        % now for species sizes and distances - these start as 3d matrices
        gen_data = sort(gen_data, 1); % sort by species number
        species_info = gen_data(:, 1:2);
        species_raw(1:data_len, :, gen) = species_info;
        
        distance_info = [gen_data(:, 1) gen_data(:,7)];
        distances_raw(1:data_len, :, gen) = distance_info;

        
        gen = gen + 1; % empty line - new generation
        gen_data = [];
    else
        data = str2double(strsplit(line, ' '));
        gen_data = [gen_data; data];
    end
    line = fgetl(input);
end
distances = process_distance(distances_raw);
species = process_species(species_raw);
fclose(input);

end

function [out] = process_distance(distance_mat)
%PROCESS_DISTANCE Prepares the species distance info
%Takes the 3d matrix of species numbers and counts and
%returns a 2d matrix of distances, one column per generation
shape = size(distance_mat);
n_species = max(max(distance_mat(:,1,:)));
out = zeros(n_species,shape(3));
for i=1:shape(3)
    gen_count = zeros(n_species,1);
    gen_data = distance_mat(:,:,i);
    % for each generation, we will put in the distance of the species
    for j=1:n_species
        count = gen_data(gen_data(:,1) == j, 2);
        if isempty(count)
            count = nan;
        end
        gen_count(j) = count;
    end
    out(:,i) = gen_count;
end
end

function [out] = process_species(species_mat)
%PROCESS_SPECIES Prepares the species count info
%Takes the 3d matrix of species numbers and counts and
%returns a 2d matrix of cumulative population sums, one columns per generation,
%so that a graph like the one in the NEAT paper can be plotted.

shape = size(species_mat);
n_species = max(max(species_mat(:,1,:)));
out = zeros(n_species,shape(3));
for i=1:shape(3)
    gen_count = zeros(n_species,1);
    gen_data = species_mat(:,:,i);
    cum_sum = 0;
    % for each generation, we will put in the distance of the species
    for j=1:n_species
        count = gen_data(gen_data(:,1) == j, 2);
        if isempty(count)
            gen_count(j) = NaN;
        else
            cum_sum = cum_sum + count;
            gen_count(j) = cum_sum;
        end
    end
    out(:,i) = 1 - gen_count/max(gen_count); % normalize 0- 1
end
end

