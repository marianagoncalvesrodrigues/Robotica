%Exercicio aula 5

% trans(l1,z) * rot(teta1,z) * trans(l2, y) * rot(-90,x) * rot(teta2,z) *
% trans(-l3,x) * trans(l4,z) * rot(teta3,z)

l1=1;
l2=1;
l3=1;
l4=1;

robotexer2 = ETS3.Tz(l1)*ETS3.Rz("q1")*ETS3.Ty(l2)*ETS3.Rx(deg2rad(-90))*...
    ETS3.Rz("q2")*ETS3.Tx(-l3)*ETS3.Tz(l4)*ETS3.Rz("q3")*ETS3.Tx(0.2);
robotexer2.teach

robotexer = ets2rbt(ETS3.Tz(l1)*ETS3.Rz("q1")*ETS3.Ty(l2)*ETS3.Rx(deg2rad(-90))*...
    ETS3.Rz("q2")*ETS3.Tx(-l3)*ETS3.Tz(l4)*ETS3.Rz("q3")*ETS3.Tx(0.2));

q = [linspace(0,pi,100)' linspace(0,pi,100)' linspace(0,pi,100)'];
r = rateControl(10);
for i = 1:size(q,1)
		robotexer.show(q(i,:),FastUpdate=true,PreservePlot=false);
		r.waitfor;
end
%%
panda = loadrobot("frankaEmikaPanda",DataFormat="row");
panda.showdetails

qr = [0 -0.3 0 -2.2 0 2 0.7854 0 0];
T = panda.getTransform(qr,"panda_hand");
printtform(T,unit="deg")
panda.show(qr);

q = [linspace(0,0,100)' linspace(0,0,100)' linspace(0,pi,100)' ...
    linspace(0,0,100)' linspace(0,0,100)' linspace(0,pi,100)' ...
    linspace(0,0,100)' linspace(0,pi,100)' linspace(0,0,100)'];

r = rateControl(10);
for i = 1:size(q,1)
		panda.show(q(i,:),FastUpdate=true,PreservePlot=false);
		r.waitfor;
end
