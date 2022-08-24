% initializating drone parameters

g = 9.8; % gravitational acceleration
L = 0.225; % length of rotors in m
m = 0.468; % mass of copter in kg
K = 2.98e-6; 
b = 0.114e-6; % centre of mass (?)
Ixx = 4.856e-3; % inertial constant of x-axis
Iyy = 4.856e-3; % inertial constant of y-axis
Izz = 8.801e-3; % inertial constant of z-axis
Ir = 3.357e-5;  % inertial constant of diagnol-axis
Ax = 0.25;
Ay = 0.25;
Az = 0.25;
Ar = 0.25;
% we are going for 3DOF - roll pitch yaw stablization

A = zeros(8, 8);
B = zeros(8, 4);
C = zeros(8, 8);
D = zeros(8, 4);

A(1, 5) = 1;
A(2, 6) = 1;
A(3, 7) = 1;
A(4, 8) = 1;

A(5, 5) = -Az/m;
A(6, 6) = -Ar/Ixx;
A(7, 7) = -Ar/Iyy;
A(8, 8) = -Ar/Izz;

A

B(5, 1) = 1/m;
B(6, 2) = 1/Ixx;
B(7, 3) = 1/Iyy;
B(8, 4) = 1/Izz;

B

C(1, 1) = 1;
C(2, 2) = 1;
C(3, 3) = 1;
C(4, 4) = 1;

C
D

%poles of the system
poles = eig(A)

%creating SS model 
sys = ss(A, B, C, D);
size(sys)

%step response of original ss model
step(30*sys(1, 1))
xlabel('roll')
ylabel('time')
% checking cotrollablity 
co = ctrb(sys);
controllable_rank = rank(co)

Q = C'*C
R = eye(4,4)

K = lqr(A, B, Q, R)

Ac = A-B*K
Bc = B;
Cc = C;
Dc = D;

% new state space after LQR
sys1 = ss(Ac, Bc, Cc, Dc)

%step(30*sys1(1, 1))
%xlabel('roll')
%ylabel('time')


