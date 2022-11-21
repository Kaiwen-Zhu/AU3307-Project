function my_p560 = create_robot(a2,d3,a3,d4)
    % set the D-H parameters
    qlim = [-pi/2, pi/2];
    theta(1) = 0; d(1) = 0;   a(1) = 0;   alpha(1) = 0;
    theta(2) = 0; d(2) = 0;   a(2) = 0;   alpha(2) = -pi/2;
    theta(3) = 0; d(3) = d3;  a(3) = a2;  alpha(3) = 0;
    theta(4) = 0; d(4) = d4;  a(4) = a3;  alpha(4) = -pi/2;
    theta(5) = 0; d(5) = 0;   a(5) = 0;   alpha(5) = pi/2;
    theta(6) = 0; d(6) = 0;   a(6) = 0;   alpha(6) = -pi/2;
    % create the links
    L(1) = Link([theta(1), d(1), a(1), alpha(1), 0], 'modified'); 
    L(2) = Link([theta(2), d(2), a(2), alpha(2), 0], 'modified'); 
    L(3) = Link([theta(3), d(3), a(3), alpha(3), 0], 'modified'); 
    L(4) = Link([theta(4), d(4), a(4), alpha(4), 0], 'modified'); 
    L(5) = Link([theta(5), d(5), a(5), alpha(5), 0], 'modified'); 
    L(6) = Link([theta(6), d(6), a(6), alpha(6), 0], 'modified');
    L(1).qlim = qlim;
    L(2).qlim = qlim;
    L(3).qlim = qlim;
    L(4).qlim = qlim;
    L(5).qlim = qlim;
    L(6).qlim = qlim;
    % create the robot
    my_p560 = SerialLink(L, 'name', 'myPuma560');
end