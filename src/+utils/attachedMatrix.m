function attached_matrix = attachedMatrix(u)
    attached_matrix = [0, -u(3), u(2);
                       u(3), 0, -u(1);
                       -u(2), u(1), 0];
end
