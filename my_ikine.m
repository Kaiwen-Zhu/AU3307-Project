function [q1,q2,q3] = my_ikine(x,y,z)
    % determine whether two floating numbers are equal
    eq = @(a,b) abs(a-b) < 1e-5;
    
    function [x,y,z] = my_fkine(q1,q2,q3)
        % calculate the forward kinematics
        x = 10 * cos(q1) * cos(q2+q3) - 100 * cos(q1) * sin(q2+q3) ...
			+ 100 * cos(q1) * cos(q2) - 20 * sin(q1);
        y = 10 * sin(q1) * cos(q2+q3) - 100 * sin(q1) * sin(q2+q3) ...
            + 100 * sin(q1) * cos(q2) + 20 * cos(q1);
        z = -10 * sin(q2+q3) - 100 * cos(q2+q3) - 100 * sin(q2);
    end

    function [q3_1, q3_2] = get_q3(x,y,z)
       rho2 = x*x + y*y + z*z;
       B = (rho2 - 20500) / 2000;
       phi1 = atan(1/(-10)) + pi;
       q3_p_phi1_m_pi = asin(-B/sqrt(101));  % q3 + phi1 - pi
       if tan(q3_p_phi1_m_pi) < -10
           q3_p_phi1_m_pi_2 = -pi - q3_p_phi1_m_pi;
       else
           q3_p_phi1_m_pi_2 = 1453;
       end
       q3_1 = q3_p_phi1_m_pi - phi1 + pi;
       q3_2 = q3_p_phi1_m_pi_2 - phi1 + pi; 
    end

    function [q2_1,q2_2] = get_q2(z,q3)
        C = cos(q3) - 10 * sin(q3) + 10;
        D = sin(q3) + 10 * cos(q3);
        phi2 = atan(D / C);
        if (-1 <= -z/10/sqrt(C*C + D*D)) && (-z/10/sqrt(C*C + D*D) <= 1)
            q2_p_phi2 = asin(-z/10/sqrt(C*C + D*D));  % q2 + phi2
            if q2_p_phi2 > 0
                q2_p_phi2_2 = pi - q2_p_phi2;
            else
                q2_p_phi2_2 = -pi - q2_p_phi2;
            end
            q2_1 = q2_p_phi2 - phi2;
            q2_2 = q2_p_phi2_2 - phi2;
        else
            q2_1 = 1453;
            q2_2 = 1453;
        end
    end

    function q1 = get_q1(x,y,q2,q3)
        A = 10 * cos(q2+q3) - 100 * sin(q2+q3) + 100 * cos(q2);
        q1 = asin((y*A - 20*x) / (x*x + y*y));
    end

    [q3_1,q3_2] = get_q3(x,y,z);
    for q3 = [q3_1,q3_2]
        if (q3 > -pi/2) && (q3 < pi/2)
            [q2_1,q2_2] = get_q2(z,q3);
            for q2 = [q2_1,q2_2]
                if (q2 > -pi/2) && (q2 < pi/2)
                    q1 = get_q1(x,y,q2,q3);
                    [x1,y1,z1] = my_fkine(q1,q2,q3);
                    if eq(x,x1) && eq(y,y1) && eq(z,z1)
						% [q1,q2,q3] is indeed a solution
                        return
                    end
                end
            end
        end
    end
    
    % fail to solve
    error("Solution not found!");
end