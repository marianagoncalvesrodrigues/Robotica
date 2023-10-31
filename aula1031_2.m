abb = loadrobot("abbIrb1600", DataFormat="row");
aIK = analyticalInverseKinematics(abb);
abbIKFcn = aIK.generateIKFunction("ikIRB1600")

A = [1 -0.5 0.7]; 
B = [1 0.5 0.7];
C = [1 -0.5 0.2];
D = [1 0.5 0.2];

R= so3(deg2rad(-30), "roty");
via = R.transform([A; B; C; D]);

A=via(1,:)
B=via(2,:)
C=via(3,:)
D=via(4,:)

TA = se3(eul2rotm([0 pi/2 0]), A);
TC = se3(eul2rotm([0 pi/2 0]), C);
TB = se3(eul2rotm([0 pi/2 0]), B);
TD = se3(eul2rotm([0 pi/2 0]), D);

solA = abbIKFcn(TA.tform)
solB = abbIKFcn(TB.tform)
solC = abbIKFcn(TC.tform)
solD = abbIKFcn(TD.tform)

waypts = [solA(1,:)' solB(1,:)' solC(1,:)' solD(1,:)' solA(1,:)'];
t = 0:0.02:10;

[q,qd,qdd,t] = trapveltraj(waypts,numel(t),EndTime=2);
plot(q(2, :), q(3, :), ".-")

r = rateControl(50);
i=1
while(1) 
    abb.show(q(:,i)',FastUpdate=true,PreservePlot=false);
    r.waitfor;
    if i == size(q,2)
        i=0
    end
    i=i+1;
end
%%
for i = 1:size(q,2)
    trajT(i) = se3(abb.getTransform(q(:,i)',"tool0"));
end
p = trajT.trvec;
plot3(q(1,:), q(2,:), q(3,:))
