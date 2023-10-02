a1=1
a2=1
e = ETS2.Rz("q1")*ETS2.Tx(a1)*ETS2.Rz("q2")*ETS2.Tx(a2);
e.fkine([deg2rad(30) deg2rad(40) 0 0])

%%
close all
hold on
%Equações obtidas algebricamente

L1=1
L2=1

r = rateControl(10);
q = [linspace(0,2*pi,100)' linspace(0,2*pi,100)'];
for i = 1:size(q,1)
    x=sin(q(i,1))
    y=cos(q(i,2))
    theta2rad = acos((x^2+y^2-L1^2-L2^2)/(2*L1*L2));
    theta1rad = atan(y/x)-atan(L2*sin(theta2rad)/(L1+L2*cos(theta2rad)));
    
    e1 = ETS2.Rz("q1")*ETS2.Tx(a1);
    e2 = ETS2.Rz("q1")*ETS2.Tx(L1)*ETS2.Rz("q2")*ETS2.Tx(L2);
    t1=e1.fkine([theta1rad])
    t2=e2.fkine([theta1rad theta2rad])
    % %posição em Y
    % posY2=cos(theta1rad)*L1 + cos(theta2rad-theta1rad)*L2;
    % posY1=cos(theta1rad)*L1;
    % 
    % %posição em x
    % posX2 = sin(theta1rad)*L1 - sin(theta2rad-theta1rad)*L2;
    % posX1= sin(theta1rad)*L1;
                
    %line([0 posX1 posX2],[0 posY1 posY2])
    line([0 t1(1,3) t2(1,3)],[0 t1(2,3) t2(2,3)])
    
    
    %myRobot.show(q(i,:),FastUpdate=true,PreservePlot=false);
    r.waitfor;
end
%%
%Utilizando as matrizes de rotação e translação obtemos a matriz homogênea do atuador do robô em relação a coordenada do mundo:
 syms a1 a2 real
 e = ETS2.Rz("q1")*ETS2.Tx(a1)*ETS2.Rz("q2")*ETS2.Tx(a2);

%Utilizando o comando de cinemática direta em função de variáveis simbólica q1 e q2 obtemos as função de cinemática direta geral:
 syms q1 q2 real
 TE = e.fkine([q1 q2])

%Como utilizaremos somente as equações referentes à posição x e y do atuador isolamos as posições (1,2) e (1,3) da matriz homogênea:
 transl = TE(1:2,3)';
 syms x y real
 e1 = x == transl(1)
 e2 = y == transl(2)

%Resolvemos estas equações em relação a q1 e q2
 [s1,s2] = solve([e1 e2],[q1 q2])
%Resultando em s1 e s2 que são equações em função de x e y
%Note que s2 possui duas equações
 %length(s2)

%Substituindo os valdores de a1 e a2 na equação de cinemática inversa temos:
 subs(s2(2),[a1 a2],[1 1])

%Comparando com o exercício do slide anterior substituimos a1=1, a2=1, q1=30 e q2=40 temos as coordenadas de x e y do atuador:
 xfk = eval(subs(transl(1), [a1 a2 q1 q2],[1 1 deg2rad(30) deg2rad(40)]))
 yfk = eval(subs(transl(2), [a1 a2 q1 q2],[1 1 deg2rad(30) deg2rad(40)]))

%Utilizando as coordenadas do slide anterior para encontrar os ângulos de cada junta temos:

 q1r = eval(subs(s1(2),[a1 a2 x y],[1 1 xfk yfk]));
 q1 = rad2deg(q1r)

 q2r = eval(subs(s2(2),[a1 a2 x y],[1 1 xfk yfk]));
 q2 = rad2deg(q2r)





