%a mass block connect a wall by a spring and damper
%with spring coefficient being k = 1N/m
%     damper coeffieient being B = 0.5N^2/m
%input is a force acted on mass block
%mass: m = 1kg
%dynamics: mx'' = f - bx' - kx
%control goal: make system stable at fixed point

A = [0 1; -1 -0.5];
n = size(A, 1);
B = [0;1];
p = size(B, 2);
C = [1 0];
D = 0;

%discretization interval
Ts = 0.1;
%continuos to discretization
sys_d = c2d(ss(A,B,C,D), Ts);

A = sys_d.a;
B = sys_d.b;
C = sys_d.c;

%initial state
x0 = [1 ; 0];
x = x0;
%initail input
u0 = 0;
u = u0;

steps = 100;

%states history
x_history = zeros(n, steps);
x_history(:,1) = x;

%input history
u_history = zeros(p,steps);
u_history(:,1) = u;

%--------------------weight design--------------%
%running weight n x n
Q = [1 0; 0 1];
%end value weight n x n
S = [1 0; 0 1];
%input weight p x p
R = 1;

F = F1_LQR_Gain(A, B, Q, R, S);

for k = 1:steps
    %calculate system input
    u = -F*x;
    %calculate respose
    x = A*x + B*u;
    
    x_history(:, k+1) = x;
    u_history(:, k+1) = u;
end

subplot(2,1,1);
for i = 1:n
    plot(x_history(i,:));
    hold;
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






