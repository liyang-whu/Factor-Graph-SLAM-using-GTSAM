function cov_matrix = info2Cov(info_vector)

% 2D case- there should be 6 elements to form upper triangle of 3X3 matrix
if numel(info_vector) == 6
    info_matrix =  [info_vector(1) info_vector(2) info_vector(3);
                    info_vector(2) info_vector(4) info_vector(5);
                    info_vector(3) info_vector(5) info_vector(6)];

% 3D case- there should be 21 elements to form upper triangle of 6X6 matrix
elseif numel(info_vector) == 21
    info_matrix = zeros(6,6);
    info_matrix(1,1:6) = info_vector(1:6);
    info_matrix(2,2:6) = info_vector(7:11);
    info_matrix(3,3:6) = info_vector(12:15);
    info_matrix(4,4:6) = info_vector(16:18);
    info_matrix(5,5:6) = info_vector(19:20);
    info_matrix(6,6) = info_vector(21);
    info_matrix = info_matrix + info_matrix' - diag(diag(info_matrix));
end

cov_matrix = inv(info_matrix);
cov_matrix = chol(cov_matrix,'lower');
end

