
%% Exercício 1
%-----------------------------------------
% Join  |  theta | d  | a  | alfa | sigma |
%  0    |    q0  | 0  | 0  |   0  |  0    |
%  1    |    q1  | 50 | 0  |   0  |  0    |
%  2    |    q2  | 0  | 0  | -90  |  0    |
%  3    |    q3  | 0  | 50 |   0  |  0    |
%  4    |    q4  | 0  | 50 | -90  |  0    |
%  5    |    q5  | 5  | 0  |   0  |  0    |
%-----------------------------------------
a0 = 0; alpha0 = 0; d0 = 0; theta0 = "q0";
a1 = 0; alpha1 = 0; d1 = 50; theta1 = "q1";
a2 = 50; alpha2 = -pi/2; d2 = 0; theta2 = "q2";
a3 = 50; alpha3 = 0; d3 = 0; theta3 = "q3";
a4 = 0; alpha4 = -pi/2; d4 = 0; theta4 = "q4";
a5 = 0; alpha5 = 0; d5 = 5; theta5 = "q5";

link0 = rigidBody("link0");
link0.Joint = rigidBodyJoint("joint0","revolute");
link0.Joint.setFixedTransform([a0 alpha0 d0 0],"dh");

link1 = rigidBody("link1");
link1.Joint = rigidBodyJoint("joint1","revolute");
link1.Joint.setFixedTransform([a1 alpha1 d1 0],"dh");

link2 = rigidBody("link2");
link2.Joint = rigidBodyJoint("joint2","revolute");
link2.Joint.setFixedTransform([a2 alpha2 d2 0],"dh");

link3 = rigidBody("link3");
link3.Joint = rigidBodyJoint("joint3","revolute");
link3.Joint.setFixedTransform([a3 alpha3 d3 0],"dh");

link4 = rigidBody("link4");
link4.Joint = rigidBodyJoint("joint4","revolute");
link4.Joint.setFixedTransform([a4 alpha4 d4 0],"dh");

link5 = rigidBody("link5");
link5.Joint = rigidBodyJoint("joint5","revolute");
link5.Joint.setFixedTransform([a5 alpha5 d5 0],"dh");

myRobot = rigidBodyTree(DataFormat="row");
myRobot.addBody(link0,myRobot.BaseName);
myRobot.addBody(link1, link0.Name);
myRobot.addBody(link2,link1.Name);
myRobot.addBody(link3,link2.Name);
myRobot.addBody(link4,link3.Name);
myRobot.addBody(link5,link4.Name);

myRobot.showdetails
myRobot.show
%% Definindo os parametros DH (Denavit-Hartenberg) para cada junta do robo
% Cada linha da matriz dh_parameters corresponde a um elo (junta) do robo
% A matriz dh_parameters tem quatro colunas: a, alpha, d e theta
% a: Comprimento do elo
% alpha: angulo entre x_i-1 e x_i ao redor de z_i-1 (angulo de torcao)
% d: distancia entre x_i-1 e x_i ao longo de z_i-1
% theta: angulo de rotacao da junta (variavel de junta).
T = eye(4); % matriz de transformacao homogenea T como matriz identidade 4x4
for i = 1:size(dh_parameters, 1) % matrizes de transformacao homogenea para cada junta e multiplicacao para obter a transformacao total do efetuador
    a = dh_parameters(i, 1);
    alpha = dh_parameters(i, 2);
    d = dh_parameters(i, 3);
    theta = dh_parameters(i, 4);
    
    % Matriz de transformacao homogenea A para a i-esima junta
    A = [
        cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta);
        sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta);
        0, sin(alpha), cos(alpha), d;
        0, 0, 0, 1
    ];
    
    % Multiplicacao da matriz de transformacao A com a matriz T
    T = T * A;
    
    % Exibicao da matriz de transformacao homogenea da junta atual
    fprintf('Matriz de transformação homogênea da Junta %d:\n', i);
    disp(A);
    fprintf('\n');
end
%% Exercicio 3
% Exibicao da matriz de transformacao homogenea total do efetuador em funcao dos angulos das juntas
fprintf('Matriz de transformacao homogenea do atuador em função dos angulos das juntas:\n');
disp(T);
%% Exercicio 4 e 5
% Criacao de um ETS3 que descreve a cinematica direta do robo com base nos angulos das juntas
e = ETS3.Rz("q1")*ETS3.Tz(50)*ETS3.Ry("q2")*ETS3.Tx(50)*ETS3.Ry("q3")*ETS3.Tx(50)*ETS3.Ry("q4")*ETS3.Tx(5)*ETS3.Ry("q5");
% Definicao de pontos de referencia A, B e C para um trajeto
A=[50 0 70];
B=[50 30 20];
C=[50 -30 20];
waypts = [A' B' C' A']; % Concatenacao dos waypoints para criar uma matriz waypts
t = 0:0.05:10; % t de 0 a 10 segundos com incremento de 0.05 s
[q2, qd2, qdd2] = trapveltraj(waypts, numel(t), EndTime=10); %trajetoria de velocidade trapezoidal (q2, qd2 e qdd2) para os waypoints
r=rateControl(1000000); %controlador de taxa para controlar a exibicao do robo
% Inicializacao de variaveis de iteracao e configuracao inicial das juntas q
i = 1;
q = [0 0 0 0 0];
%ajuste das juntas do robo e exibir a trajetoria planejada
while(1)
     % Otimizacao dos angulos das juntas para se aproximarem dos angulos desejados (q2)
     q = fminsearch(@(q) norm([se3(e.fkine(q)).trvec se3(e.fkine(q)).eul]-[q2(1:3,i)' 0 0 0]), q); 
     % Exibicao do robo na configuracao atual
     e.plot(q,'jointdiam',1)
     r.waitfor;
     % Verificacao de termino da trajetoria
     if i == size(q2, 2)
         i=0;
     end
     i = i + 1;
end
