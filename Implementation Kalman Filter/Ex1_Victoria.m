close all, clear all

%% TP1 Kalman 11/01


N = 500;
X_init = [1;2];
Xk(:,1)=X_init;

% modelisation
A=eye(2);
C= eye(2);
H=eye(2);

%initialisation
%P=eye(2);
P = [1000000000 0 ; 0 10000000000];

X_k_sachant_k=zeros(2,N);
X_k_sachant_k(:,1)=[3;4]; 

Y_mes= C*X_init+ randn(2,N); %mesures bruitées


% Cas ou on considere un bruit dynamique
zeta = zeros(2,N);
%zeta = randn(2,N);

%covariance des bruits
Q= [1e-7 0 ;0  1e-7 ];
R= eye(2,2);

I2= eye(2,2);


% Pour stocker 
error_innovation = zeros(2,N);

%500 colonnes de 2 lignes
Composante_diag_Kalman = zeros(2,N);

% diag de P
Composante_diag_P = zeros(2,N);

%%%%%%%% KALMAN
for k=2:N

    %PREDICTION pour l'état
    %mise a jour de l'état estimé
    X_k_sachant_k(:,k)= A*X_k_sachant_k(:,k-1) + H*zeta(:,k);

    %Cacul de la cov de l'erreur d'estimation
    P= A*P*A' + H*Q*H'; 
    
    %CORRECTION
    K= P*C'*inv(C*P*C'+R);

    X_k_sachant_k(:,k)=X_k_sachant_k(:,k)+K*(Y_mes(:,k)-C*X_k_sachant_k(:,k));
    P=(I2-K*C)*P;

    % error
    estimated_pose =X_k_sachant_k(:,k);
    error_innovation(:,k) =  [estimated_pose(1) - X_init(1);estimated_pose(2) - X_init(2)];
    
    Composante_diag_Kalman(:,k) = [K(1);K(4)];
    Composante_diag_P(:,k)= [P(1);P(4)];
end
% plot


% POS ESTIMATE
figure(1)
subplot(1,2,1); hold on; grid on;
plot(X_k_sachant_k(1,:));
plot(Y_mes(1,:));
plot(ones(1,N)*X_init(1));
legend("estime","mesure","reelle");
xlabel("numero mesure")
ylabel("position x")


subplot(1,2,2);hold on; grid on;
plot(X_k_sachant_k(2,:));
plot(Y_mes(2,:));
plot(ones(1,N)*X_init(2));
legend("estime","mesure","reelle");
xlabel("numero mesure")
ylabel("position y")

title("Pos estimé sur N mesures");


figure(2); hold on;
plot(error_innovation(1,:))
plot(error_innovation(2,:))
title("Erreur innovation");
xlabel("numero mesure")
ylabel("Erreur")

figure(3);hold on;
plot(Composante_diag_Kalman(1,:));
plot(Composante_diag_Kalman(2,:));
title(" Composante_diag_Kalman ")

figure(4)
plot(Composante_diag_P(1,:));
plot(Composante_diag_P(2,:));
title("Composante diag de la matrice P")

