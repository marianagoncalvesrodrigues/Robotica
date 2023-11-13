%Robo Aranha aula 13/11
%Trabalho
%q1 em x
%q2 e q2 em y

L1 = 0.5;
L2 = 0.5;
e = ETS3.Rx("q1")*ETS3.Ty(-0.2075)*ETS3.Tx(0.3375)*...
    ETS3.Tz(-0.2850)*ETS3.Ry("q2")*ETS3.Tz(-0.8840)*ETS3.Ry("q3")*ETS3.Tz(-0.6840);
%Modifica de ETS para o sistema de corpo rígido de robôs
leg = ets2rbt(e);
se3(leg.getTransform([0 0 0],"link8")).trvec
leg.show([deg2rad(-10) deg2rad(45) deg2rad(-45)]);

%%
xf = 30/100; xb = -xf; y = -20.75/100; zu = -120/100; zd = -140/100;

via = [xf y zd
xb y zd
xb y zu
xf y zu
xf y zd];

x = mstraj(via,[],[3 0.25 0.5 0.25],[],0.01,0.1);
qcycle = ikineTrajNum(leg,se3(eye(3),x),"link8", ...
weights=[0 0 0 1 1 1]);

r = rateControl(200);
while(1)
    for i = 1:size(qcycle,1)
        leg.show(qcycle(i,:),FastUpdate=true,PreservePlot=false);
        r.waitfor;
    end
end
%%
W = 100/100; L = 200/100;
Tflip = se3(pi,"rotz");

legs = [ ...
rbtTform(leg,se3(eye(3),[L/2 W/2 0])*Tflip), ...
rbtTform(leg,se3(eye(3),[-L/2 W/2 0])*Tflip), ...
rbtTform(leg,se3(eye(3),[L/2 -W/2 0])), ...
rbtTform(leg,se3(eye(3),[-L/2 -W/2 0]))];

r = rateControl(200);
i=0;
while(1)
%for i = 1:5000
    i=i+1;
 	legs(1).show(gait(qcycle,i,0,false),FastUpdate=true,PreservePlot=false); hold on;
	 legs(2).show(gait(qcycle,i,100,false),FastUpdate=true,PreservePlot=false);
 	legs(3).show(gait(qcycle,i,200,true),FastUpdate=true,PreservePlot=false);
 	legs(4).show(gait(qcycle,i,300,true),FastUpdate=true,PreservePlot=false); hold off;
    r.waitfor;
end

function q = gait(cycle, k, phi, flip)
 k = mod(k+phi-1,size(cycle,1))+1;
 q = cycle(k,:);
 if flip
    q(1) = -q(1); % for left-side legs
 end
end

