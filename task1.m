clear; close all; clc;

% create the robot with a2=100,d3=20,a3=10,d4=100
my_p560 = create_robot(100,20,10,100);

% plot the robot
my_p560.plot(zeros(1,6));
hold on;

% sample parameters randomly to get the workspace
N = 20000;  % num of samples
for i = 1:N
    T = my_p560.fkine(-pi/2 + rand(1, 6) * pi);
    pos = transl(T);
    x(i) = pos(1); 
    y(i) = pos(2); 
    z(i) = pos(3);
end

% plot the positions
scatter3(x,y,z, 1, 'r', 'filled');


