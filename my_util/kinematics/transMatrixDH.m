function T = transMatrixDH(a, d, alpha, theta)
%transMatrixDH: Homogenous transformation matrix define by DH
%order: Rot(x, alpha)*Trans(x, a)*Trans(z, d)*Rot(z,theta)

T = [cos(theta)                                          -sin(theta)                                  0                             a;
      sin(theta)*cos(alpha)                cos(theta)*cos(alpha)        -sin(alpha)          -d*sin(alpha);
      sin(theta)*sin(alpha)                 cos(theta)*sin(alpha)          cos(alpha)           d*cos(alpha);
          0                                                             0                                           0                               1           ];  