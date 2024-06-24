clear all;

%mass of whole car
M = 500e-3;
%mass of wheel
m = 50e-3;
%length of center of mass
L = 45e-3;
g = 9.8;
%radius of wheel
r = 19.5e-3;
%Length of car
Len = 98e-3;
%Width of car
Wid = 86e-3;
%Height of car
Hei = 39e-3;
%moment of inertial of wheel
I = m*r^2;
%moment of inertial of car(axis of rotation is axle, pitch, Y-axis)
J =  M*(Wid^2+Hei^2)/12 + M*(L - r)^2;

q = J*M+(J+M*L^2)*(2*m+2*I^2/r^2);

A = [0 1 0 0; 0 0 -M*L^2*g/q 0; 0 0 0 1; 0 0 M*L*g*(M+2*m+2*I/r^2)/q 0];
B = [0 0; (J+M*L^2+M*L*r)*I/(q*r^2) (J+M*L^2+M*L*r)*I/(q*r^2); 0 0; I*(M*L/r+M+2*m+2*L/r^2)/(q*r) I*(M*L/r+M+2*m+2*L/r^2)/(q*r)];
C = [0 0 1 0];
D = 0;

n = size(A,1);
p = size(B,2);

%discretization interval
Ts = 0.05;
%continuos to discretization
sys_d = c2d(ss(A,B,C,D), Ts);

A = sys_d.a;
B = sys_d.b;
C = sys_d.c;

%define desired states transition matrix
AD = eye(n);

%initial state
x0 = [0 ; 0; pi/180; 0];
x = x0;

%desired state
xd = [0; 0; 0; 0];

%construct augmented matrix
xa = [x; xd];
Aa = [A zeros(n); zeros(n) AD];
Ba = [B; zeros(n,p)];
Ca = [eye(n) -eye(n)];

%initail input
u0 = [0;0];
u = u0;

steps = 400;

%states history
x_history = zeros(n, steps);
x_history(:,1) = x;

%input history
u_history = zeros(p,steps);
u_history(:,1) = u;

%--------------------weight design--------------%
%running weight n x n
Q = [2 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
%end value weight n x n
S = [2 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
%input weight p x p
R = [200 0; 0 200];

%construct augmented matrix
Sa = transpose(Ca)*S*Ca;
Qa = transpose(Ca)*Q*Ca;

F = F1_LQR_Gain(Aa, Ba, Qa, R, Sa);

for k = 1:steps
    %calculate system input
    u = -F*xa;
    %calculate respose
    x = A*x + B*u;
    %disturbance
%     if k == 100
%         x(1) = x(1) + 0.1;
%     end
    %update augmented matrix
    xa = [x; xd];
    x_history(:, k+1) = x;
    u_history(:, k) = u;
end

subplot(2,1,1);
hold on;
for i = 1:n
    plot(x_history(i,:));
end
legend(num2str((1:n)', 'x %d'));
xlim([1, steps]);

subplot(2,1,2);
for i = 1:p
    stairs (u_history(i,:));
    hold;
end
legend(num2str((1:p)', 'u %d'));
xlim([1, steps]);
grid on


