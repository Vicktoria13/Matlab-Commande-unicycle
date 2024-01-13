clear all, close all,


%% Paramètres 

L = 0.1; %m
r =0.05;
l1 = 0.5; % 10 cm devant
l2 = 0;


T= 10; % Durée de la simulation
Te= 0.1 ; % Période d'échantillonnage
t = 0:Te:T;

K1 = 2;
K2 =K1;
K = [K1 0; 
     0 K2];

% position initiale (centre du unicycle)

x0 = 1;
y0 = 3;
theta0 = 0;
phi0_1 = 0;
phi0_2 = 0;

% 1 ligne : vecteur configuration initiale
q0 = [x0, y0, theta0, phi0_1,phi0_2]; 

% On genere la trajectoire 2D sur une duree T
[Xr,Yr] = generate_trajectoire(length(t)-1);


% Position du point P contraint lors de la simulation
xp = zeros (1,length(t)-1); % 101 element
yp = zeros (1,length(t)-1);

% init : position du point contraint a t=0
xp(1) = x0 + cos(theta0)*l1-sin(theta0)*l2;
yp(1) = y0 + sin(theta0)*l1+cos(theta0)*l2;

q=q0;

erreur_suivi_X=[];
erreur_suivi_Y= [];


u1 = [];
u2 =[];


u1_max =2 ;
u2_max = 2;
%% SUIVI DE TRAJECTOIRE %%
for i=2:length(t)-1 % commencer a i=2 pour calcul de l'erreur
  
    % derivée de la consigne Xr_point

    Xr_point=[(Xr(i)-Xr(i-1))/ Te; 
              (Yr(i)-Yr(i-1))/ Te];

    %erreur entre consigne et mesure
    error = [Xr(i)-xp(i-1); 
             Yr(i)-yp(i-1)];

    V = Xr_point + K*error;

    Matrice_uv = [cos(q(3)) -l1*sin(q(3));
                  sin(q(3))  l1*cos(q(3))];

    u = (Matrice_uv)\V; % vitesse d'avance + rotation

    %saturation des vitesses
    if u(1) > u1_max 
        u(1) = u1_max;
    end

    if u(2) > u2_max 
        u(2) = u2_max;
    end
    u1 = [u1;u(1)];
    u2 = [u2;u(2)];


    [temps,q_apres_simu]= ode45(@(tcont, q) modele_cinematique(tcont,q,u), [0 Te], q);
    q=q_apres_simu(end,:);

    %ajout d'une quantification
    q = 0.01*round(q*100) ;


    %MAJ
    xp(i)= q(1) + cos(q(3))*l1-sin(q(3))*l2;
    yp(i) = q(2) + sin(q(3))*l1+cos(q(3))*l2;
    
    %erreur 

    erreur_suivi_X = [erreur_suivi_X;error(1)];
    erreur_suivi_Y = [erreur_suivi_Y;error(2)];

end

%% plt


figure(1);
plot(xp,yp,'LineWidth',2)
hold on; grid on;
plot(Xr,Yr,'x','LineWidth',2)
xlabel('X');ylabel('Y');title("asservissement d'un point P contraint : suivi de trajectoire");
legend('Trajectoire réelle ','Trajectoire désirée');


figure(2); 
plot(erreur_suivi_X)
hold on;grid on;
plot(erreur_suivi_Y)
title("erreurs de suivi");
legend('erreur suivi X ','erreur suivi Y');


figure(3); 
subplot(1,2,1)
plot(xp)
hold on;grid on;
plot(Xr)
title("réponse a la consigne en X");
legend('consigne X ','réponse X');
txt = {'erreur asymptotique = ',abs(Xr(end-1) - yp(end-1))};
text(4,0.5,txt)


subplot(1,2,2)
plot(yp)
hold on;grid on;
plot(Yr)
title("réponse a la consigne en Y");
legend('consigne Y ','réponse Y');
txt = {'erreur asymptotique = ',abs(Yr(end-1) - yp(end-1))};
text(4,0.5,txt)



%% commande

figure(4); 
plot(u1)
hold on;grid on;
plot(u2)
title("evolution des commandes");
legend('u1 : vitesse d avance ','u2 : vitesse de rotation');

txt = {'vitesse u1 maximale atteinte = ',max(u1)};
text(10,0.5,txt)

txt = {'vitesse u2 maximale atteinte = ',max(u2)};
text(30,1,txt)


