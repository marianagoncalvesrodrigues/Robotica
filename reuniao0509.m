%q=(x,y) %objetivo

xo=1;
yo=5;
xc=0;
yc=0;
%inicio da iteraçao
Kv=0.1;
v= Kv*sqrt((xo-xc)^2 + (yo-yc)^2)
teta = atan((yo-yc)/(xo-xc))
graus = rad2deg(teta)

teta = deg2rad(20)
L =1;
R1=L/ tan(teta)
eixo = 0.12;
vai = v/ (R1+eixo/2)
vae = v/ (R1-eixo/2)
%link: https://edisciplinas.usp.br/pluginfile.php/3280265/mod_resource/content/1/Aula%203%20-%20SEM5911%20Robo%CC%81tica%20Mo%CC%81vel.pdf
%cinematica direta para atualizar x e y do carrinho
%fim da iteraçaõ
%grafico - carrinho fazendo a trajetória para encontrar o objetivo
