function out = extendcat(a, b)
%from  http://stackoverflow.com/questions/15519685/matlab-add-vectors-not-in-the-same-length-to-a-matrix
if (~isempty(a) && ~isempty(b))
    diff = length(a) - length(b);

    if diff > 0
        b = [b; repmat(b(end,:), diff, 1)];
    else
        a = [a; repmat(a(end, :), -diff, 1)];
    end
end
out = [a,b];

end