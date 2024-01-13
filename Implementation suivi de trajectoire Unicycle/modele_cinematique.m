%% Modèle cinématique direct

function q_point = modele_cinematique(~,q, u)
    
    Longueur_L = 0.2; %m
    rayon_roue =0.1;
    % u => [u1 u2 ].T
    % q => [ x y theta phi_droite_point, phi_gauche_point]
    x_point = u(1)*cos(q(3));
    y_point = u(1)*sin(q(3));
    theta_point = u(2);

    tmp  = Longueur_L*u(2);
    phi_droite_point = (u(1)+tmp)/rayon_roue;
    phi_gauche_point = (u(1)-tmp)/rayon_roue;
    
    q_point = [x_point;y_point;theta_point;phi_droite_point;phi_gauche_point];

end


