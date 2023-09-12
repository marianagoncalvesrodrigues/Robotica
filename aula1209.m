
%link.Joint.setFixedTransform([a alpha d theta],"dh");

link1 = rigidBody("link1");
link1.Joint = rigidBodyJoint("joint1","revolute");
link1.Joint.setFixedTransform([0 deg2rad(-90) 660 0],"dh");

link2 = rigidBody("link2");
link2.Joint = rigidBodyJoint("joint2","revolute");
link2.Joint.setFixedTransform([432 0 50 0],"dh");

link3 = rigidBody("link3");
link3.Joint = rigidBodyJoint("joint3","revolute");
link3.Joint.setFixedTransform([0 pi/2 0 0],"dh");

link4 = rigidBody("link4");
link4.Joint = rigidBodyJoint("joint4","revolute");
link4.Joint.setFixedTransform([0 -pi/2 432 0],"dh");

link5 = rigidBody("link5");
link5.Joint = rigidBodyJoint("joint5","revolute");
link5.Joint.setFixedTransform([0 pi/2 0 0],"dh");

link6 = rigidBody("link6");
link6.Joint = rigidBodyJoint("joint6","revolute");
link6.Joint.setFixedTransform([0 0 0 0],"dh");


myRobot = rigidBodyTree(DataFormat="row");
myRobot.addBody(link1,myRobot.BaseName);
myRobot.addBody(link2,link1.Name);
myRobot.addBody(link3,link2.Name);
myRobot.addBody(link4,link3.Name);
myRobot.addBody(link5,link4.Name);
myRobot.addBody(link6,link5.Name);
myRobot.showdetails
%%
q = [linspace(0,pi,100)' linspace(0,-2*pi,100)' linspace(0,-2*pi,100)' linspace(0,-2*pi,100)' linspace(0,-2*pi,100)' linspace(0,-2*pi,100)'];
r = rateControl(10);
for i = 1:size(q,1)
	myRobot.show(q(i,:),FastUpdate=true,PreservePlot=false);
	r.waitfor;
end
