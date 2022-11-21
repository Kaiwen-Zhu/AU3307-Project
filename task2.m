clear; close all; clc;

% create the robot with a2=200,d3=20,a3=10,d4=200
my_p560 = create_robot(200,20,10,200);

% change the base
offset = 100;
my_p560.base = [-offset, 0, 0];

% plot the obstacle
center = [100, 0, 50];
size = [200, 30, 150];
origin = center - size/2;
vertex_index = [0,0,0; 0,0,1; 0,1,0; 0,1,1; 1,0,0; 1,0,1; 1,1,0; 1,1,1];
% compute the positions of eight vertices
vertex_pos = origin + vertex_index.*size;  
% specify the six facets
facet=[1,2,4,3; 1,2,6,5; 1,3,7,5; 2,4,8,6; 3,4,8,7; 5,6,8,7]; 
patch('Vertices',vertex_pos, 'Faces',facet, 'EdgeColor','white');
hold on;
xlim([-300, 300]);
ylim([-300, 300]);
zlim([-300, 300]);

% compute the joint variables corresponding to the four anchor points
pini = [100, 100, 10];  % the start point
pmid1 = [100, 100, 200];  % the upper middle point
pmid2 = [100, -100, 200];  % the lower middle point
pend = [100, -100, 10];  % the end point
qini = zeros(1,6); qend = zeros(1,6);
qmid1 = zeros(1,6); qmid2 = zeros(1,6); 
[qini(1),qini(2),qini(3)] = my_ikine(pini(1)+offset, pini(2), pini(3));
[qmid1(1),qmid1(2),qmid1(3)] = my_ikine(pmid1(1)+offset, pmid1(2), pmid1(3));
[qmid2(1),qmid2(2),qmid2(3)] = my_ikine(pmid2(1)+offset, pmid2(2), pmid2(3));
[qend(1),qend(2),qend(3)] = my_ikine(pend(1)+offset, pend(2), pend(3));

% interpolate the joint variables smoothly
t0 = [0 : 0.05 : 1]';
q1 = mtraj(@tpoly, qini, qmid1, t0);
q2 = mtraj(@tpoly, qmid1, qmid2, t0);
q3 = mtraj(@tpoly, qmid2, qend, t0);
q = [q1; q2(2:end,:); q3(2:end,:)];

% plot the path of the end effector
T = my_p560.fkine(q);
p = transl(T);
scatter3(pini(1), pini(2), pini(3), 100, 'r', 'filled');
scatter3(pend(1), pend(2), pend(3), 100, 'r', 'filled');
plot3(p(:,1), p(:,2), p(:,3), 'LineWidth',5);
my_p560.plot(q);

waitfor(gcf);

% plot the joint variables
hold on;
t = [0 : 0.05 : 3]';
for i = 1:6
    plot(t, q(:,i), ...
    'DisplayName',strcat('\theta_',num2str(i)), 'LineWidth',1);
end
legend;
ylim([-pi/2, pi/6]);
yticks(-pi/2 : pi/12 : pi/6);
yticklabels({'-90^{\circ}','-75^{\circ}','-60^{\circ}','-45^{\circ}', ...
    '-30^{\circ}','-15^{\circ}','0^{\circ}','15^{\circ}','30^{\circ}'});