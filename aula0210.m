%%
hold on
%Equações obtidas algebricamente

L1=1
L2=1

r = rateControl(10);
q = [linspace(0,1,100)' linspace(0,1,100)'];
for i = 1:size(q,1)
    x=q(i,1)
    y=q(i,2)
    theta2rad = acos((x^2+y^2-L1^2-L2^2)/(2*L1*L2));
    theta1rad = atan(y/x)-atan(L2*sin(theta2rad)/(L1+L2*cos(theta2rad)));
    
    e1 = ETS2.Rz("q1")*ETS2.Tx(a1);
    e2 = ETS2.Rz("q1")*ETS2.Tx(L1)*ETS2.Rz("q2")*ETS2.Tx(L2);
    t1=e1.fkine([theta1rad])
    t2=e2.fkine([theta1rad theta2rad])
    % %posição em Y
    % posY2=cos(theta1rad)*L1 + cos(theta2rad-theta1rad)*L2;
    % posY1=cos(theta1rad)*L1;
    % 
    % %posição em x
    % posX2 = sin(theta1rad)*L1 - sin(theta2rad-theta1rad)*L2;
    % posX1= sin(theta1rad)*L1;
                
    %line([0 posX1 posX2],[0 posY1 posY2])
    line([0 t1(1,3) t2(1,3)],[0 t1(2,3) t2(2,3)])
    
    
    %myRobot.show(q(i,:),FastUpdate=true,PreservePlot=false);
    r.waitfor;
end
