%% Exercicio 2
close all
Ry = rotmy(deg2rad(20))'*[1 2 3]'
Rtotal = rotmz(deg2rad(30))'*rotmy(deg2rad(20))'*[1 2 3]'%0.9252 1.7752 3.1611
