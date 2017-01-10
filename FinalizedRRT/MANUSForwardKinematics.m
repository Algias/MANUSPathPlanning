function T6 = MANUSForwardKinematics(J)
theta1 = J(1);
c1 = cos(theta1);
s1 = sin(theta1);
theta2 = J(2);
c2 = cos(theta2);
s2 = sin(theta2);
theta3 = J(3);
c3 = cos(theta3);
s3 = sin(theta3);
theta4 = J(4);
c4 = cos(theta4);
s4 = sin(theta4);
theta5 = J(5);
c5 = cos(theta5);
s5 = sin(theta5);
theta6 = J(6);
c6 = cos(theta6);
s6 = sin(theta6);
L2 = .400;
d2 = .100;
d4 = .326;
%T6 = a1*a2*a3*a5*a6
%An = transformation matrix relating the coordinate frame of link n to
%coordinate frame of link n-1.
a1 = [c1 0 s1 0;
      s1 0 -c1 0;
      0 1 0 0;
      0 0 0 1];
a2 = [ c2 -s2 0 L2*c2;
       s2 c2 0 L2*s2;
       0 0 1 -d2;
       0 0 0 1];
a3 = [c3 0 s3 0;
      s3 0 -c3 0;
      0 1 0 0;
      0 0 0 1];
a4 = [c4 0 s4 0;
      s4 0 -c4 0;
      0 1 0 d4;
      0 0 0 1];
a5 = [c5 0 -s5 0;
      s5 0 c5 0;
      0 -1 0 0;
      0 0 0 1];
a6 = [c6 -s6 0 0;
      s6 c6 0 0;
      0 0 1 0;
      0 0 0 1];
%   t6 = [nx ox ax px;
%         ny oy ay py;
%         nz oz az pz;
%         0 0 0 1;]
T6 = a1*a2*a3*a4*a5*a6;
%where vector p describes coordinates of TCP w/resp to base frame
% and three unit orientation vectors n, o and a the orientation of the
% gripper with respect to the base frame. The elements of T6 =:
%ox = -((c1*c23*c4+s1*s4)*c5+c1*s23*s5)*s6+(-c1c23s4+s1c4)c6
%oy = -((s1c23c4-c1s4)c5+s1s23s5)s6+(-s1c23s4-c1c4)c6
%oz = -(s23c4c5-c23s5)s6-s23s4c6
%ax = -(c1c23c4+s1s4)s5+c1s23c5
%ay = -(s1c23c4-c1s4)s5+s1s23c5
%az = -s23c4s5-c23c5
%nx = oyaz-ozay
%ny = -oxaz+ozax
%nz = oxay-oyax
%px = c1s23d4 +c1L2c2-s1d2
%py = s1s23d4+s1L2c2+c1d2
%pz = -c23d4+L2s2
%where s23 and c23 denote sin(theta2 +theta3) and cos(theta2 +theta3)

end