function A = adjacency_sigmoid(P, delta, omega)
    
    [n_dim, n] = size(P);
    A = zeros(n);

    for i = 1:n
        for j = i+1:n
            d_ij = norm(P(:,1) - P(:,j));
            a_ij = 1/(1+exp(omega *(delta - d_ij)));
            A(i,j) = a_ij;
            A(j,i) = a_ij;
        end
    end

end