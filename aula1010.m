%%
%Resolvendo o problema de minimização da equação cinemática direta em relação a posição desejada temos:
 e = ETS2.Rz("q1")*ETS2.Tx(1)*ETS2.Rz("q2")*ETS2.Tx(1);
 pstar = [0.6 0.7]; % Desired position
 q = fminsearch(@(q) norm(se2(e.fkine(q)).trvec-pstar),[0 0])
 printtform2d(e.fkine(q),unit="deg")
%%
%Executando com valores do exemplo anterior
 pstar = [1.2080 1.4397];
 q = fminsearch(@(q) norm(se2(e.fkine(q)).trvec-pstar),[0 0])
 rad2deg(q)

%%
%Para o robô planar de 3 segmentos encontre as equações de cinemática inversa utilizando o MATLAB. 
%Com as equações de cinemática inversa calcule os ângulos para o atuador na posição (85, 42).
%Faça uma animação com o atuador do robô realizando uma trajetória circular de raio 20 em torno do 
%ponto (70,70)

e1 = ETS2.Rz("q1")*ETS2.Tx(50)*ETS2.Rz("q2")*ETS2.Tx(40)*ETS2.Rz("q3")*ETS2.Tx(30);

r= rateControl(10);
qq = [linspace(0,6*pi,1000)'];
for i = 1:size(qq,1)
	pstar = [70+20*cos(qq(i)) 70+20*sin(qq(i))]; % Desired position
    q = fminsearch(@(q) norm(se2(e1.fkine(q)).trvec-pstar),q)
    rad2deg(q)
    e1.plot(q, 'workspace', [0 100 0 100 0 1])
	r.waitfor;
end





