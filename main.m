x = -22.5884 ;       % X coordinate of end effector in cm
y = -2.9059 ;       % Y coordinate of end effector in cm
theta = 0.6446*180/pi ;    % orientation of end effector in degree
O2O = 300;      % distance between consecutive O
B2B = 100;      % distance between consecutive B
l = 100;        % link length Ai to Bi

r = B2B/(2*cosd(30));
R = O2O/(2*cosd(30));
O = [0;0];

% Coordinates of E
E = [x;y];

% Coordinates of Bi
B1 = r*[cosd(theta+210);sind(theta+210)] + E;
B2 = r*[cosd(theta-30);sind(theta-30)] + E;
B3 = r*[cosd(theta+90);sind(theta+90)] + E;

% Coordinates of Oi
O1 = R*[cosd(210);sind(210)];
O2 = R*[cosd(-30);sind(-30)];
O3 = R*[cosd(90);sind(90)];

% Coordinates of Ai and actuation length qi (Oi to Ai)
[A1,q1] = igm(O1,O2,B1,l);
A1_1 = [A1(1),A1(2)]; A1_2 = [A1(3),A1(4)];
q1_1 = q1(1); q1_2 = q1(2);
[A2,q2] = igm(O2,O3,B2,l);
A2_1 = [A2(1),A2(2)]; A2_2 = [A2(3),A2(4)];
q2_1 = q2(1); q2_2 = q2(2);
[A3,q3] = igm(O3,O1,B3,l);
A3_1 = [A3(1),A3(2)]; A3_2 = [A3(3),A3(4)];
q3_1 = q3(1);q3_2 = q3(2);

% Printing output
fprintf('A1_1 = '); disp(A1_1)
fprintf('A1_2 = '); disp(A1_2)
fprintf('q1 = '); disp(q1)
fprintf('A2_1 = '); disp(A2_1)
fprintf('A2_2 = '); disp(A2_2)
fprintf('q2 = '); disp(q2)
fprintf('A3_1 = '); disp(A3_1)
fprintf('A3_2 = '); disp(A3_2)
fprintf('q3 = '); disp(q3)
fprintf('B1 = '); disp(B1')
fprintf('B2 = '); disp(B2')
fprintf('B3 = '); disp(B3')


% Plotting part
figure(1)
hold on
plot(O(1),O(2),'ko');
plot(O1(1),O1(2),'r.');
plot(O2(1),O2(2),'r.');
plot(O3(1),O3(2),'r.');

plot(x,y,'g+');
plot(B1(1),B1(2),'b.');
plot(B2(1),B2(2),'b.');
plot(B3(1),B3(2),'b.');

% plot(A_1a(1),A_1a(2),'k.');
% plot(A_2a(1),A_2a(2),'k.');
% plot(A_3a(1),A_3a(2),'k.');

plot(A1(1),A1(2),'k.');
plot(A2(1),A2(2),'k.');
plot(A3(1),A3(2),'k.');

plot(A1(3),A1(4),'k.');
plot(A2(3),A2(4),'k.');
plot(A3(3),A3(4),'k.');

plot([O1(1),O2(1)],[O1(2),O2(2)],'r-')
plot([O2(1),O3(1)],[O2(2),O3(2)],'r-')
plot([O3(1),O1(1)],[O3(2),O1(2)],'r-')

plot([B1(1),B2(1)],[B1(2),B2(2)],'b-')
plot([B2(1),B3(1)],[B2(2),B3(2)],'b-')
plot([B3(1),B1(1)],[B3(2),B1(2)],'b-')

plot([B1(1),A1(1)],[B1(2),A1(2)],'k-')
plot([B2(1),A2(1)],[B2(2),A2(2)],'k-')
plot([B3(1),A3(1)],[B3(2),A3(2)],'k-')

plot([B1(1),A1(3)],[B1(2),A1(4)],'k--')
plot([B2(1),A2(3)],[B2(2),A2(4)],'k--')
plot([B3(1),A3(3)],[B3(2),A3(4)],'k--')



function [A,q] = igm(Oi,On,B,l)
Oi_B = Oi-B;
m = norm(Oi_B);
v = Oi_B/m;
u = (Oi-On)/norm(Oi-On);

beta = abs(acos(dot(u,v)));
alpha = asin(m*sin(beta)/l);
gamma = pi-alpha-beta;

alpha_n = abs(pi-alpha);
gamma_n = pi-alpha_n-beta;


q_a = l*sin(gamma)/sin(beta);

q_b = l*sin(gamma_n)/sin(beta);

A_a = -q_a*u + Oi;
A_b = -q_b*u + Oi;

    A = [A_a,A_b];
    q = [q_a,q_b];
    
if norm(A_a-B) > l
    fprintf("\n ERROR \n Position out of reach..!!\n\n")
end
    
end





