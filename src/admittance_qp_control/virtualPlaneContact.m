function contact = virtualPlaneContact(pPen, vPen, planePoint, cfg)
%VIRTUALPLANECONTACT Kelvin-Voigt unilateral plane-contact model.
%
% The contact normal points from free space into the obstacle. Positive
% signed distance therefore means penetration.

    n = cfg.contactNormal(:);
    n = n / norm(n);

    signedDistance = dot(n, pPen - planePoint);
    penetration = max(0, signedDistance);

    if penetration > 0
        penetrationRate = dot(n, vPen);
        forceMagnitude = cfg.environmentStiffness * penetration ...
                       + cfg.environmentDamping * penetrationRate;
        forceMagnitude = max(0, forceMagnitude);
    else
        penetrationRate = 0;
        forceMagnitude = 0;
    end

    contact = struct;
    contact.penetration = penetration;
    contact.penetrationRate = penetrationRate;
    contact.forceMagnitude = forceMagnitude;
    contact.forceVector = -forceMagnitude * n;
    contact.signedDistance = signedDistance;
end
