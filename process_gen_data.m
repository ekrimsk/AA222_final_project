function [means, mins, maxs, stds] = process_gen_data(genData); 
    numGens = numel(genData); 
    means = zeros(numGens, 1);
    mins = zeros(numGens, 1);
    maxs = zeros(numGens, 1);
    stds = zeros(numGens, 1);

    % NOTE - could add 'quartile' data 
    for k = 1:numGens
        fits = genData{k}.fitness;
        means(k) = mean(fits);
        mins(k) = min(fits);
        maxs(k) = max(fits);
        stds(k) = std(fits);
    end 
end 







