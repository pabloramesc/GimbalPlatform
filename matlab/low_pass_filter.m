function output = low_pass_filter(input, gain)
    N = size(input, 1); % number of samples
    dim = size(input, 2); % number of columns (dimensions)
    prev = zeros(1, dim);
    output = zeros(N, dim);
    for k = 1:N
        output(k,:) = gain*prev + (1.0-gain)*input(k,:);
        prev = output(k,:);
    end
end