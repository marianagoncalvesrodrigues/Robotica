
link0 = rigidBody("link0");
link0.Joint = rigidBodyJoint("joint0","fixed");
link0.Joint.setFixedTransform([0 1 0 0], "dh");

link05 = rigidBody("link05");
link05.Joint = rigidBodyJoint("joint05","revolute");
link05.Joint.setFixedTransform([0 0 0 0], "dh");

link1 = rigidBody("link1");
link1.Joint = rigidBodyJoint("joint1","fixed");
link1.Joint.setFixedTransform([0 1 0 pi/2], "dh");

link2 = rigidBody("link2");
link2.Joint = rigidBodyJoint("joint2","revolute");
link2.Joint.setFixedTransform([0 0 1 0], "dh");

link3 = rigidBody("link3");
link3.Joint = rigidBodyJoint("joint3","revolute");
link3.Joint.setFixedTransform([0 1 0 0], "dh");

link4 = rigidBody("link4");
link4.Joint = rigidBodyJoint("joint4","fixed");
link4.Joint.setFixedTransform([0 0 0.1 0], "dh");


myRobot = rigidBodyTree(DataFormat="row");
myRobot.addBody(link0,myRobot.BaseName);
myRobot.addBody(link05,link0.Name);
myRobot.addBody(link1,link05.Name);
myRobot.addBody(link2,link1.Name);
myRobot.addBody(link3,link2.Name);
myRobot.addBody(link4,link3.Name);
myRobot.showdetails
myRobot.show
% myRobot.show([.5 1 1.5]);
% myRobot.getTransform([.5 1 1.5], "link4")
%_______________________________________
%---- theta  |   d   |   a   |  alpha  |
%        0   |  L1   |   0   |    0    |  {0}
%       q1   |   0   |   0   |    0    |  {0,5}
%        0   |  L2   |   0   |   90    |  {1}
%       q2   |   0   |  L3   |    0    |  {2}
%       q3   |  L4   |   0   |    0    |  {3}
%        0   |   0   |  L5   |    0    |  {4}
%_______________________________________
