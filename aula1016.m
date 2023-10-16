
%JUNTA 1
%TRANSLACAO EM Z 660MM
%ROTACAO EM Z Q1
%JUNTA 2
%ROTACAO EM X 90 GRAUS
%ROTACAO EM Z Q2
%TRANSLACAO EM X 432MM
%JUNTA 3
%ROTACAO EM Z Q3
%TRANSLACAO EM X 432
%JUNTA 4
%ROTACAO EM X Q4
%ROTACAO EM Z Q5
%ROTACAO EM X Q6
%TRANSLACAO EM X 50MM
e2=ETS3.Tz(660)*ETS3.Rz("Q1")*...
    ETS3.Rx(pi/2)*ETS3.Rz("Q2")*...
    ETS3.Tx(432)*ETS3.Rz("Q3")*...
    ETS3.Tx(432)*ETS3.Rx("Q4")*...
    ETS3.Rz("Q5")*ETS3.Rx("Q6")*...
    ETS3.Tx(50)
q=[0 0 0 0 0 0]
r= rateControl(10);
qq = [linspace(0,6*pi,1000)'];
for i = 1:size(qq,1)
	pstar = [400 200*cos(qq(i)) 700+200*sin(qq(i))]; % Desired position
    q = fminsearch(@(q) norm(se3(e2.fkine(q)).trvec-pstar),q)
    rad2deg(q)
    e2.plot(q, 'workspace', [-1000 1000 -1000 1000 -1000 1000])
	r.waitfor;
end
