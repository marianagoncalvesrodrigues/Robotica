%Robo Aranha aula 13/11
%Trabalho
%q1 em x
%q2 e q2 em y

L1 = 0.5;
L2 = 0.5;
e = ETS3.Rx("q1")*ETS3.Ty(-0.2075)*ETS3.Tx(0.3375)*...
    ETS3.Tz(-0.2850)*ETS3.Ry("q2")*ETS3.Tz(-0.8840)*ETS3.Ry("q3")*ETS3.Tz(-0.6840);
e2 = ETS3.Rx("q1")*ETS3.Ty(0.2075)*ETS3.Tx(0.3375)*...
    ETS3.Tz(-0.2850)*ETS3.Ry("q2")*ETS3.Tz(-0.8840)*ETS3.Ry("q3")*ETS3.Tz(-0.6840);
%Modifica de ETS para o sistema de corpo rígido de robôs
leg = ets2rbt(e);
leg2 = ets2rbt(e2);
figure(1)
%se3(leg.getTransform([0 0 0],"link8")).trvec
leg.show([deg2rad(-10) deg2rad(45) deg2rad(-45)]);
figure(2)
leg2.show([deg2rad(-10) deg2rad(45) deg2rad(-45)]);

%% qcycle frente-esquerda - trajetória da perna para frente
xf = (40+33.75)/100; xb = (-40+33.75)/100; y = -20.75/100; zu = -120/100; zd = -140/100;
via = [xf y zd
xb y zd
xb y zu
xf y zu
xf y zd];
x = mstraj(via,[],[3 0.25 0.5 0.25],[],0.01,0.1);
qcycle = ikineTrajNum(leg,se3(eye(3),x),"link8", ...
weights=[0 0 0 1 1 1]);
%% qcycle2 frente-direita - trajetória da perna para frente
xf = (-40+33.75)/100; xb = (40+33.75)/100; y = 20.75/100; zu = -120/100; zd = -140/100;
via = [xf y zd
xb y zd
xb y zu
xf y zu
xf y zd];
x = mstraj(via,[],[3 0.25 0.5 0.25],[],0.01,0.1);
qcycle2 = ikineTrajNum(leg2,se3(eye(3),x),"link8", ...
weights=[0 0 0 1 1 1]);
%% qcycle3 para o lado frente - trajetória da perna para o lado
xf = (33.75)/100; xb = (33.75)/100; yf = (30-20.75)/100; y = -30/100-20.75/100; zu = -120/100; zd = -140/100;
via = [xf yf zd
xb y zd
xb y zu
xf yf zu
xf yf zd];
x = mstraj(via,[],[3 0.25 0.5 0.25],[],0.01,0.1);
qcycle3 = ikineTrajNum(leg,se3(eye(3),x),"link8", ...
weights=[0 0 0 1 1 1]);
%% qcycle4 para o lado tras - trajetória da perna para o lado
xf = (33.75)/100; xb = (33.75)/100; yf = (-30-20.75)/100; y = 30/100-20.75/100; zu = -120/100; zd = -140/100;
via = [xf yf zd
xb y zd
xb y zu
xf yf zu
xf yf zd];
x = mstraj(via,[],[3 0.25 0.5 0.25],[],0.01,0.1);
qcycle4 = ikineTrajNum(leg2,se3(eye(3),x),"link8", ...
weights=[0 0 0 1 1 1]);
%% qcycle5 para o circulo - trajetória da perna 
%raio do robo raiz(100^2+50^2) = 112
%y vai de -30,56 a 35,79
%x 112*cos(10 graus) - 100 e 112*cos(50 graus) - 100
%x vai de 10,2 a -28
xf = (33.75)/100; xb = (33.75)/100; yf = (30-20.75)/100; y = -30/100-20.75/100; zu = -120/100; zd = -140/100;
via = [xf yf zd
xb y zd
xb y zu
xf yf zu
xf yf zd];
x = mstraj(via,[],[3 0.25 0.5 0.25],[],0.01,0.1);
qcycle3 = ikineTrajNum(leg,se3(eye(3),x),"link8", ...
weights=[0 0 0 1 1 1]);
%% qcycle6 para o circulo - trajetória da perna 
xf = 112*cos(deg2rad(10))-100; xb = (33.75)/100; yf = (-30-20.75)/100; y = 30/100-20.75/100; zu = -120/100; zd = -140/100;
via = [xf yf zd
xb y zd
xb y zu
xf yf zu
xf yf zd];
x = mstraj(via,[],[3 0.25 0.5 0.25],[],0.01,0.1);
qcycle4 = ikineTrajNum(leg2,se3(eye(3),x),"link8", ...
weights=[0 0 0 1 1 1]);
%%
r = rateControl(200);
while(1)
    for i = 1:size(qcycle,1)
        leg.show(qcycle(i,:),FastUpdate=true,PreservePlot=false);
        r.waitfor;
    end
end

%% animação do robô com as quatro pernas realizando os movimentos para frente
close all
W = 100/100; L = 200/100;
Tflip = se3(pi,"rotz");
legs = [ ...
rbtTform(leg,se3(eye(3),[L/2 -W/2 0])), ...         %frente
rbtTform(leg,se3(eye(3),[-L/2 W/2 0])*Tflip), ...   %atras
rbtTform(leg2,se3(eye(3),[-L/2 -W/2 0])*Tflip), ... %frente
rbtTform(leg2,se3(eye(3),[L/2 W/2 0]))];            %atras
r = rateControl(200);

%% para frente
i=0;
while(1)
%for i = 1:5000
    i=i+1;
 	legs(1).show(gait(qcycle2,i,0,false),FastUpdate=true,PreservePlot=false); hold on;
	 legs(2).show(gait(qcycle,i,100,false),FastUpdate=true,PreservePlot=false);
 	legs(3).show(gait(qcycle,i,200,true),FastUpdate=true,PreservePlot=false);
 	legs(4).show(gait(qcycle2,i,300,true),FastUpdate=true,PreservePlot=false); hold off;
    r.waitfor;
end

%% Para o lado esquerda
i=0;
while(1)
%for i = 1:5000
    i=i+1;
 	legs(1).show(gait(qcycle4,i,0,false),FastUpdate=true,PreservePlot=false); hold on;
	 legs(2).show(gait(qcycle3,i,100,false),FastUpdate=true,PreservePlot=false);
 	legs(3).show(gait(qcycle4,i,200,true),FastUpdate=true,PreservePlot=false);
 	legs(4).show(gait(qcycle3,i,300,true),FastUpdate=true,PreservePlot=false); hold off;
    r.waitfor;
end

%% Para o lado direita
i=0;
while(1)
%for i = 1:5000
    i=i+1;
 	legs(1).show(gait(qcycle3,i,0,false),FastUpdate=true,PreservePlot=false); hold on;
	 legs(2).show(gait(qcycle4,i,100,false),FastUpdate=true,PreservePlot=false);
 	legs(3).show(gait(qcycle3,i,200,true),FastUpdate=true,PreservePlot=false);
 	legs(4).show(gait(qcycle4,i,300,true),FastUpdate=true,PreservePlot=false); hold off;
    r.waitfor;
end
