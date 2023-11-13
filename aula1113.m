%Robo Aranha aula 13/11
%Aula 11 slide 20


L1 = 0.5;
L2 = 0.5;
e = ETS3.Rz("q1")*ETS3.Rx("q2")*ETS3.Ty(-L1)* ...
 	ETS3.Rx("q3")*ETS3.Tz(-L2);
%Modifica de ETS para o sistema de corpo rígido de robôs
leg = ets2rbt(e);
se3(leg.getTransform([0 0 0],"link5")).trvec
leg.show;

%%
xf = 0.25; xb = -xf; y = -0.25; zu = -0.1; zd = -0.25;

via = [xf y zd
xb y zd
xb y zu
xf y zu
xf y zd];

x = mstraj(via,[],[3 0.25 0.5 0.25],[],0.01,0.1);
qcycle = ikineTrajNum(leg,se3(eye(3),x),"link5", ...
weights=[0 0 0 1 1 1]);

r = rateControl(200);

for i = 1:size(qcycle,1)
    leg.show(qcycle(i,:),FastUpdate=true,PreservePlot=false);
    r.waitfor;
end

W = 0.5; L = 1;
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

