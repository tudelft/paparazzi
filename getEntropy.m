function entropy = getEntropy(p_dist)
entropy = 0;
for i = 1:size(p_dist,2)
    if(p_dist(i) > 0)
        entropy = entropy - p_dist(i) * log2(p_dist(i));
    end
end