%% Aula 23/10 - slide 31 aula 7
abb = loadrobot("abbIrb1600",DataFormat="row");
aIK = analyticalInverseKinematics(abb);
abbIKFcn = aIK.generateIKFunction("ikIRB1600")

q = [0 0 0 0 0 0];
r = rateControl(1000);
qq = linspace(0, 6*pi, 1000)';
for i = 1:size(qq,1)
pstar = [1 0.40*cos(qq(i)) 0.70+0.40*sin(qq(i))]; %posição desejada
tgtPose = trvec2tform(pstar)*eul2tform([0 pi/2 0]);
q = abbIKFcn(tgtPose, q(1,:))

abb.show(q(1,:))
r.waitfor;
end
