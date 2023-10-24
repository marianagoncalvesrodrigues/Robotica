%% Aula 24/10 - slides 3-5 
cobra = loadrvcrobot("cobra");
TE = se3(deg2rad([180 0 30]),"eul","XYZ",[0.4 -0.3 0.2]);

cobraHome = cobra.homeConfiguration;
cobraIK = inverseKinematics(RigidBodyTree=cobra);
weights = ones(1,6);
rng(0); % obtain repeatable results
[qsol,solinfo] = cobraIK("link4",TE.tform,weights,cobraHome)

T4 = cobra.getTransform(qsol, "link4");
printtform(T4, unit="deg", mode="xyz")
%%
weights = [0 0 1 1 1 1];
[qsol,solinfo] = cobraIK("link4",TE.tform,weights,cobraHome)
%% cinematica direta
T = cobra.getTransform(qsol, "link4")
printtform(T,  unit="deg", mode="xyz");
%% Super-atuado
panda = loadrobot("frankaEmikaPanda",DataFormat="row");
TE = se3(trvec2tform([0.7 0.2 0.1]))* ...
se3(oa2tform([0 1 0],[0 0 -1]));

pandaHome = panda.homeConfiguration;
pandaIK = inverseKinematics(RigidBodyTree=panda);
rng(0); % obtain repeatable results
%A solução da cinemática inversa é :
qsol = pandaIK("panda_hand",TE.tform,ones(1,6),pandaHome)
%A solução pode ser verificada:
handT = panda.getTransform(qsol,"panda_hand");
printtform(handT,mode="xyz",unit="deg")
panda.show(qsol);


