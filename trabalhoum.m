%% Parte 1
%-----------------------------------------
% Join  |  theta | d  | a  | alfa | sigma |
%  1    |    q1  | 50 | 0  |   0  |  0    |
%  2    |    q2  | 0  | 0  |  -90 |  0    |
%  3    |    q3  | 0  | 50 |   0  |  0    |
%  4    |    q4  | 0  | 50 |   0  |  0    |
%  5    |    q5  | 5  | 0  |  -90 |  0    |
%-----------------------------------------
clc
clear all
t1 = "q1";
d1 = 50; 
a1 = 0; 
alpha1 = 0; 

t2 = "q2";
d2 = 0;
a2 = 0; 
alpha2 = -pi/2;

t3 = "q3";
d3 = 0;
a3 = 50; 
alpha3 = 0;

t4 = "q4";
d4 = 0;
a4 = 50; 
alpha4 = 0;

t5 = "q5";
d5 = 5;
a5 = 0; 
alpha5 = -pi/2;  

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
myRobot.addBody(link1,myRobot.BaseName);

myRobot.addBody(link2,link1.Name);
myRobot.addBody(link3,link2.Name);
myRobot.addBody(link4,link3.Name);
myRobot.addBody(link5,link4.Name);

myRobot.showdetails
myRobot.show

%% Parte 2

a     = [a1,         a2,     a3,     a4,     a5];
alpha = [alpha1, alpha2, alpha3, alpha4, alpha5];
d     = [d1,         d2,     d3,     d4,     d5];
theta = [0,           0,      0,      0,      0]; 

%Inicializa a matriz de transformação MatrizTransf com uma matriz de identidade 4x4. Esta matriz acumulará as transformações ao longo das juintas
MatrizTransf = eye(4); 
%Armazena as matrizes de transformação para cada junta. Isso é útil para visualizar as transformações intermediárias.
MatrizesTransf = cell(1, numel(a));

for i = 1:numel(a)
    T = [
        cos(theta(i)), -sin(theta(i)) * cos(alpha(i)), sin(theta(i)) * sin(alpha(i)), a(i) * cos(theta(i));
        sin(theta(i)), cos(theta(i)) * cos(alpha(i)), -cos(theta(i)) * sin(alpha(i)), a(i) * sin(theta(i));
        0, sin(alpha(i)), cos(alpha(i)), d(i);
        0, 0, 0, 1;
    ];
    MatrizTransf = MatrizTransf * T;
    MatrizesTransf{i} = MatrizTransf;
end


for i = 1:numel(MatrizesTransf)
    fprintf('Matriz de transformação para a junta %d:\n', i);
    disp(MatrizesTransf{i});
end

%% Parte 3

MatrizesTransf = cell(1, numel(a) + 1);
MatrizesTransf{numel(a) + 1} = MatrizTransf;
for i = 1:numel(a)
    T = [
        cos(theta(i)), -sin(theta(i)) * cos(alpha(i)), sin(theta(i)) * sin(alpha(i)), a(i) * cos(theta(i));
        sin(theta(i)), cos(theta(i)) * cos(alpha(i)), -cos(theta(i)) * sin(alpha(i)), a(i) * sin(theta(i));
        0, sin(alpha(i)), cos(alpha(i)), d(i);
        0, 0, 0, 1;
    ];
    MatrizTransf = MatrizTransf * T;
    MatrizesTransf{i} = MatrizTransf;
end
disp('Matriz de transformação para o atuador:');
disp(MatrizesTransf{end});

%% Parte 4
clear all
close all

syms a1 a2 real
e = ETS2.Tx(50) * ETS2.Rz("q1") * ETS2.Tx(50) * ETS2.Tx(50) * ETS2.Rz("q2") * ETS2.Tx(5);

syms q1 q2 q3 real
TE = e.fkine([q1 q2 q3])
transl = TE(1:2,3)';

syms x y real
e1 = x == transl(1)
e2 = y == transl(2)

[s1,s2,s3] = solve([e1 e2],[q1 q2 q3])

length(s2)

%% Parte 5
% Definição das variáveis simbólicas
syms a1 a2 real
e = ETS2.Tx(50) * ETS2.Rz("q1") * ETS2.Tx(50) * ETS2.Tx(50) * ETS2.Rz("q2") * ETS2.Tx(5);

% Define as equações para a cinemática inversa
syms q1 q2 q3 real
TE = e.fkine([q1 q2 q3]);
transl = TE(1:3, 3);

% Define as coordenadas dos vértices do triângulo
A = [50; 0; 70];
B = [50; 30; 20];
C = [50; -30; 20];

% Define as equações para a cinemática inversa
e1 = A == transl(1);  % Substitua para B e C em iterações posteriores
e2 = B == transl(2);
e3 = C == transl(3);

% Resolve o sistema de equações numericamente
[s1, s2, s3] = vpasolve(e1, [q1, q2, q3]);

%
q=[0 0 0 0 0 0]
r= rateControl(10);
qq = [linspace(0,6*pi,1000)'];
for i = 1:size(qq,1)
   pstar = [1 .70+0.20*cos(qq(i)) .70+0.20*sin(qq(i))]; % Desired position
q = fminsearch(@(q) norm(se3(e.fkine(q)).trvec-pstar),q)
rad2deg(q)
e.plot(q, 'workspace', [-1 1 -1 1 -1 1])
   r.waitfor;
end
%%
e =ETS2.Tx(50) * ETS2.Rz("q1") * ETS2.Tx(50) * ETS2.Tx(50) * ETS2.Rz("q2") * ETS2.Tx(5) * ETS2.Rz("q3");

q = [0 0 0];
r = rateControl(10);
g = linspace(0, pi, 100)';
for i = 1:size(g,1)
pstar = [50 0 -30]; %posição desejada
q = fminsearch(@(q) norm([se3(e.fkine(q)).trvec se3(e.fkine(q)).eul]-pstar),q);
rad2deg(q)
%axis([0 100 0 100]
e.plot(q, 'workspace', [-1000 1000 -1000], "scale", 0.1)

r.waitfor;
end


