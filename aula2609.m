
link1 = rigidBody("link0");
link1.Joint = rigidBodyJoint("joint0","fixed");
link1.Joint.setFixedTransform([0 0 10 0], "dh");

link2 = rigidBody("link1");
link2.Joint = rigidBodyJoint("joint1","fixed");
link2.Joint.setFixedTransform([0 pi/2 0 0], "dh");

link3 = rigidBody("link2");
link3.Joint = rigidBodyJoint("joint2","revolute");
link3.Joint.setFixedTransform([50 0 0 0], "dh");

link4 = rigidBody("link3");
link4.Joint = rigidBodyJoint("joint3","revolute");
link4.Joint.setFixedTransform([40 0 0 0], "dh");

link5 = rigidBody("link4");
link5.Joint = rigidBodyJoint("joint4","revolute");
link5.Joint.setFixedTransform([30 0 0 0], "dh");

myRobot = rigidBodyTree(DataFormat="row");
myRobot.addBody(link1,myRobot.BaseName);
myRobot.addBody(link2,link1.Name);
myRobot.addBody(link3,link2.Name);
myRobot.addBody(link4,link3.Name);
myRobot.addBody(link5,link4.Name);
myRobot.showdetails

myRobot.show([.5 1 1.5]);
myRobot.getTransform([.5 1 1.5], "link4")
%_______________________________________
%---- theta  |   d   |   a   |  alpha  |
%       0    |  10   |   0   |    0    |  {1}
%       q1   |  0    |   0   |   90    |  {2}
%       q2   |  0    |  50   |   90    |  {3}
%       q3   |  0    |  40   |    0    |  {4}
%       0    |  0    |  30   |    0    |  {5}
%_______________________________________
