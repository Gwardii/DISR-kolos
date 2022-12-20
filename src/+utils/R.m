function rot_matrix = R(phi, u)
    rot_matrix = u' * u * (1 - cos(phi)) + eye(3) * cos(phi) + utils.attachedMatrix(u) * sin(phi);
end
