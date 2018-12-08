clear;
close all;

[d, v_ang, var]=read_rosbag_data();
d=abs(d);
THRESHOLD=0.055;
k=2;

while(abs(d(1,k))>THRESHOLD && abs(d(2,k))>THRESHOLD  && abs(d(3,k))>THRESHOLD )
    k=k+1;
end
tinicial=k-1;

if(d(1,k)<d(2,k) && d(1,k)<d(3,k))
    start=1;
else if(d(2,k)<d(3,k))
        start=2;
    else
        start=3;
    end
end

switch start
    case 1
        landmarks=[2; 3];
    case 2 
        landmarks=[1;3];
    case 3
        landmarks=[1;2];
end

while(abs(d(landmarks(1,1),k)) >THRESHOLD && abs(d(landmarks(2,1),k)) >THRESHOLD)
    k=k+1;
end   
tfinal=k-1;

if(abs(d(landmarks(1,1),k))>abs(d(landmarks(2,1),k)))
    finish=landmarks(2,1);
    estimate=landmarks(1,1);
else
    finish=landmarks(1,1);
    estimate=landmarks(2,1);
end

start=1;
finish=2;
estimate=3;
[lm,e]=trilat([-d(finish,tinicial:1:tfinal); zeros(1,tfinal-tinicial+1)] ,d(estimate,tinicial:1:tfinal).^2, d(estimate,tfinal).^2);

 LM(:,start)=[-d(finish,tinicial),0];
 LM(:,finish)=[0,0];
 LM(:,estimate)=lm;
 
 figure;
 hold on;
 scatter(LM(1,1), LM(2,1));
  scatter(LM(1,2), LM(2,2));
   scatter(LM(1,3), LM(2,3));
   
   LANDMARKS=3;
   SIZE=10+2*LANDMARKS;
   x=zeros(SIZE,length(d)-tfinal);
   z=zeros(6,length(d)-tfinal);
 %INICIALIZAÇÃO
%Posição e velocidade do veículo a (0,0) e segunda landmark a (0,0)
%Segunda Landmark Inicializada a (D,0)
%Terceira Landmark inicializada com o resultado da trilateração
x(:, 1) = [0; 0; 0; 0; LM(1,1); LM(2,1); LM(1,2); LM(2,2); LM(1,3); LM(2,3) ; 0 ; 0 ; 0 ; 0 ; 0 ; 0];
%Varânicia da posiçao , velocidade do veículo e da segunda landmark é nula (assume-se que está 100% correta)
%Variancia do x primeira landmark assume-se a variancia das medições
%Variância da 3ª landmark corresponde ao maior desvio quadrático proveniente da trilateração
cov=diag([ 0 0 0.2 0.2  0.3 0 0 0 e*ones(1, 2) 0 0 0 0.3 0.3 0.3]);

%Matriz F
F=eye(10+2*LANDMARKS);
F(1,3)=1;
F(2,4)=1;
F(length(x(:, 1))-5,length(x(:, 1))-2)=1;
F(length(x(:, 1))-4,length(x(:, 1))-1)=1;
F(length(x(:, 1))-3,length(x(:, 1)))=1;

for t=2:1:length(d)-tfinal
    
   
   %% Prediction
   %Assume um valor de velocidades parecido ao do instante anterior
   xp(3:4)=x(3:4,t-1);
   xp(SIZE-2:SIZE)=x(SIZE-2:SIZE,t-1);
   %Itera a posição com base na velocidade assumida
   xp(1:2)=x(1:2, t-1)+xp(3:4)';
   xp(SIZE-5:SIZE-3)=x(SIZE-5:SIZE-3, t-1)+xp(SIZE-2:SIZE)';
   %As landmarks não são afetadas pelo movimento do drone
   xp(5:SIZE-6)=x(5:SIZE-6,t-1);
   
   %Atualiza a matriz das covariancias
   covp=F*cov*F'+diag([0.1 0.1 0.01 0.01 0 0 0 0 0 0 pi/15 pi/15 pi/15 0.2 0.2 0.2]);
   
   
   
   %% Update
   %Determina as dstâncias às landmarks (com erros maximos de 0.2)
   z(:,t)=[d(:,t+tfinal).^2 ; [0;0;0]];
   %Determina o S 
   S=H(xp, LANDMARKS)*covp*H(xp, LANDMARKS)' + diag([0.0001 0.0001 0.0001 0 0 0]);
   %Calcula o ganho de Kalman
   K=covp*H(xp, LANDMARKS)'*inv(S);
   %Corrige a matriz das covariâncias
   cov=covp-K*S*K';
   
   %Corrige a estimativa do vetor de estado
   %Inovation
   
   i=z(:,t)-hp(xp);
   %Filtro de outliers
   x(:,t)=xp'+(K*i);   
   %Filtro passa baixo para suavizar o sinal
   %x(1:3,t)=(xpre(1:3)+x(1:3,t-1))/2;
   %x(4:12,t)=xpre(4:12);
   
   
   %% Grafismos
   
   ref_ekf=rotation(x(SIZE-5:SIZE-3,t));
   
   %% Posições
   figure(1);
   clf;
   hold on;
   plot(x(1,t),x(2,t),'+', 'MarkerEdgeColor', 'r');
   plot(x(5,t),x(6,t),'o', 'MarkerEdgeColor', 'g');
   plot(x(7,t),x(8,t),'o', 'MarkerEdgeColor', 'g');
   plot(x(9,t),x(10,t),'o', 'MarkerEdgeColor', 'g');   
   plot(x(1,1:t),x(2,1:t),'r');
   
   legend('Posição atual (EKF)', 'Landmarks Estimadas');
   grid on;
   title(['Espaço de Estados (t=' num2str(t) ')']);
   xlabel('X1');
   ylabel('X2');
   
   %% Orientação
   figure(2);
   clf;
   hold on
   
   title(['Orientação do Drone (t=' num2str(t) ')'])
   quiver3(0,0,0,ref_ekf(1,1),ref_ekf(2,1),ref_ekf(3,1), 'r');
   quiver3(0,0,0,ref_ekf(1,2),ref_ekf(2,2),ref_ekf(3,2), 'r');
   quiver3(0,0,0,ref_ekf(1,3),ref_ekf(2,3),ref_ekf(3,3), 'r');  
   text(ref_ekf(1,1),ref_ekf(2,1),ref_ekf(3,1), 'x"');
   text(ref_ekf(1,2),ref_ekf(2,2),ref_ekf(3,2), 'y"');
   text(ref_ekf(1,3),ref_ekf(2,3),ref_ekf(3,3), 'z"');
   view(135,45);
   
   legend('Orientação estimada');
   grid on;
   set(gca,'XTick',[], 'YTick', [], 'ZTick', []);
   pause(0.0001);
   
end