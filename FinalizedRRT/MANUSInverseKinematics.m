function [J,failed] = MANUSInverseKinematics(T6,shoulderup,elbowup)
    px = T6(1,4);
    py = T6(2,4);
    pz = T6(3,4);
    
    ax = T6(1,3);
    ay = T6(2,3);
    az = T6(3,3);
    
    ox = T6(1,2);
    oy = T6(2,2);
    oz = T6(3,2);
    
    nx = T6(1,1);
    ny = T6(2,1);
    nz = T6(3,1);
    
    d2 = .100;
    d4 = .326;
    L2 = .400;
try
    if (shoulderup == 1)
        theta1 = atan2(py,px) - atan2(d2,sqrt(px^2+py^2-d2^2));
    elseif (shoulderup == 0)
        theta1 = atan2(py,px) - atan2(d2,-sqrt(px^2+py^2-d2^2));
    end
catch
    failed = 1;
    J = [];
    return
end
    c1 = cos(theta1);
    s1 = sin(theta1);
    %where - sign corresponds to left hand shoulder configuation, and +
    %being right hand shoulder configuration.
    s3 = ((c1*px+s1*py)^2+pz^2-d4^2-L2^2)/(2*d4*L2);
    if ~isreal(s3)
        J = [];
        failed = 1;
        return 
    end
    if (elbowup == 1)
        c3 = sqrt(1-s3^2);
    elseif(elbowup == 0)
        c3 = -sqrt(1-s3^2);
    end
    if ~isreal(c3)
        J = [];
        failed = 1;
        return 
    end
    %+- corresponding to elbow up / elbow down configuration.
    theta3 = atan2(s3,c3);
    %s2 = (((c1*px+s1*py)*d4*c3)+((d4*s3+L2)*pz))/(d4^2*c3^2+((d4*s3+L2)^2));
    %c2 = ((-d4*c3*pz)+((d4*s3+s2)*(c1*px+s1*py)))/(d4^2*c3^2+((d4*s3+L2)^2));
    theta2 = atan2((c1*px+s1*py)*d4*c3+(d4*s3+L2)*pz,(-d4*c3*pz)+((d4*s3+L2)*(c1*px+s1*py)));
    c23 = cos(theta2+theta3);
    s23 = sin(theta2+theta3);
    theta4 = atan2(-s1*ax+c1*ay,-c23*(c1*ax+s1*ay)-s23*az);
    s4 = sin(theta4);
    c4 = cos(theta4);
    theta5 = atan2(-c4*(c23*(c1*ax+s1*ay)+s23*az)-s4*(s1*ax-c1*ay), s23*(c1*ax+s1*ay)-c23*az);
    if (theta5 < 0)
        theta4 = theta4 + pi;
    end
    theta6 = atan2(-s4*(c23*(c1*nx+s1*ny)+s23*nz)+c4*(s1*nx-c1*ny), -s4*(c23*(c1*ox+s1*oy)+s23*oz)+c4*(s1*ox-c1*oy));
    J = [theta1,theta2,theta3,theta4,theta5,theta6];
    failed = 0;
end