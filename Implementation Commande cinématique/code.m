close all
clear all

%% AJOUTER LE PACKAGE quiverRotate si vous voulez de beaux vecteurs de vitesse

%%%%%%%%% DATA ENONCE %%%%%%%%%%%%%%%%%%%%%%%%%%%%
gamma = 0;
r = 0.05; %en m
l = 0.20; % en m

theta = 0 % question 4

% permet de mettre une fonction sur theta
R_theta = @(theta) [cos(theta) -sin(theta) 0;
            sin(theta) cos(theta)  0; 
            0 0 1]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%% Modèle cinématique :  matrice J calculée

J = [r*sqrt(3)/3 0 -r*sqrt(3)/3;
    -r/3 2*r/3 -r/3 ;
    -r/(3*l) -r/(3*l) -r/(3*l) ];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% tests question 4 %%%%%%%%%%%%%%%%% 
% on essaie de visualiser le vecteur pose connaissant le vecteur vitesse
% /exprimé dans R0 (psi point)


N_points= 1000; 
dt = 0.1;

%% Q4) cas 1 : 



%simulation dure dt*N

phi_point = [4; 4; 4];
%phi1_point = input("Rentrez phi1 point \n" )
%phi2_point = input("Rentrez phi2 point \n" )
%phi3_point = input("Rentrez phi3 point \n" )
%phi_point = [phi1_point;phi2_point;phi3_point]

% fonction que l'on tracera

pose_x = zeros(1,N_points);
pose_y = zeros(1,N_points);
pose_theta = zeros(1,N_points);


pose_x_alterne = zeros(1,N_points);
pose_y_alterne = zeros(1,N_points);

U= zeros(1,N_points);
V= zeros(1,N_points);

vitesse_R0 = inv(R_theta(0))*J * phi_point;


vecteur_temps = linspace(0,dt*N_points,N_points);
dt = vecteur_temps(2) - vecteur_temps(1);

pose_current = [0;0;0];

for temps = 1: N_points
    % temps va de 1,2...,N_points

    pose_x(temps) = pose_current(1);
    pose_y(temps) = pose_current(2);
    pose_theta(temps) = pose_current(3);

    theta_current = pose_current(3);

    vitesse = inv(R_theta(theta_current))*J*phi_point;

    pose_current(1) = vitesse(1)*dt + pose_current(1);
    pose_current(2) = vitesse(2)*dt + pose_current(2);
    pose_current(3) = vitesse(3)*dt + pose_current(3);


    if mod(temps,30)==0
        U(temps) = vitesse(1);
        V(temps) = vitesse(2);

    end

end


%%%% plot

figure(1);
quiver(pose_x,pose_y,U,V,0) 
hold on;
plot(pose_x,pose_y,'.')
grid on;
title("deplacement (x,y) du robot et quelques vecteurs vitesses associés")
legend("vecteurs vitesses associés")
subtitle("phi point = [ "+ phi_point(1) + ", "+phi_point(2)+ ", "+phi_point(3)+" ]")
xlabel('x (m) ')
ylabel('y(m)')


figure(2);
subplot(1,3,1)
plot(vecteur_temps,pose_x)
xlabel('t (s) ')
ylabel('x(m)')
title("evolution de x en fonction du temps")
subplot(1,3,2)
plot(vecteur_temps,pose_y)
xlabel('t (s) ')
ylabel('y(m)')
title("evolution de y en fonction du temps")
subplot(1,3,3)
plot(vecteur_temps,pose_theta)
xlabel('t (s) ')
ylabel('theta(deg)')
title("evolution de theta en fonction du temps")