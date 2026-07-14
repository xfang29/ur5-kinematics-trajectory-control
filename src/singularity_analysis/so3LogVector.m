function phi = so3LogVector(R)
%SO3LOGVECTOR Return the rotation-vector logarithm of a rotation matrix.
%
%   phi = so3LogVector(R)
%
% R must be a 3-by-3 rotation matrix. The output phi is a 3-by-1 vector
% whose direction is the rotation axis and whose norm is the angle in rad.

    arguments
        R (3,3) double
    end

    % Clamp the cosine argument to protect acos from floating-point drift.
    cosTheta = (trace(R) - 1) / 2;
    cosTheta = min(1, max(-1, cosTheta));
    theta = acos(cosTheta);

    veeSkew = @(S) [S(3,2); S(1,3); S(2,1)];

    if theta < 1e-8
        % First-order approximation: log(R) ~= (R - R') / 2.
        phi = 0.5 * veeSkew(R - R.');
        return;
    end

    if abs(pi - theta) < 1e-5
        % Numerically robust axis extraction near 180 degrees.
        A = (R + eye(3)) / 2;
        axis = sqrt(max(diag(A), 0));

        [~, idx] = max(axis);
        switch idx
            case 1
                axis(2) = sign(R(1,2) + R(2,1)) * axis(2);
                axis(3) = sign(R(1,3) + R(3,1)) * axis(3);
            case 2
                axis(1) = sign(R(1,2) + R(2,1)) * axis(1);
                axis(3) = sign(R(2,3) + R(3,2)) * axis(3);
            case 3
                axis(1) = sign(R(1,3) + R(3,1)) * axis(1);
                axis(2) = sign(R(2,3) + R(3,2)) * axis(2);
        end

        if norm(axis) < 1e-10
            axis = [1; 0; 0];
        else
            axis = axis / norm(axis);
        end

        phi = theta * axis;
        return;
    end

    phi = theta / (2 * sin(theta)) * veeSkew(R - R.');
end
