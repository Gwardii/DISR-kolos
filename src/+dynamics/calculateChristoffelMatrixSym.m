function C = calculateChristoffelMatrixSym(M, dq)
    dq = dq(:);

    for i = 1:height(M)

        for j = 1:width(M)
            grad_M(i, j, :) = gradient(M(i, j));
        end

    end

    for i = 1:height(M)

        for j = 1:width(M)

            for k = 1:size(grad_M, 3)
                c(k) = 0.5 * (grad_M(i, j, k) + grad_M(i, k, j) - grad_M(j, k, i));
            end

            C(i, j) = c * dq;
        end

    end

end
