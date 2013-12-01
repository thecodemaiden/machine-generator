function out = vertcat2(a, b)
%from  http://stackoverflow.com/questions/15519685/matlab-add-vectors-not-in-the-same-length-to-a-matrix
if (~isempty(a) && ~isempty(b))
    a_shape = size(a);
    b_shape = size(b);

    diff = a_shape(1) - b_shape(1);

    if diff > 0
        b = [b; nan(diff, b_shape(2))];
    else
        a = [a; nan(-diff, a_shape(2))];
    end
 
end
out = [a,b];

end