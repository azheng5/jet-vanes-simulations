function noisy_data = generate_noise(data,noise)
%generate_noise Takes in data and generates noisy data
%   data - the array you wish to make noisy
%   noise - degree of noise (ie 0.1 -> noise within 10% of the value)

data_dims = size(data);
noisy_data = data .* (1 + (-1.* noise + (randn(data_dims(1),data_dims(2)) .* 2 .* noise)));

end