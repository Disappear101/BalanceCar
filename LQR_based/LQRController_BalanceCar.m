clear all;

m = 0.5;
L = 45e-3;
g = 9.8;
A = [0 1; g/L 0]
B = [0; 1];
C = [1 0];
D = 0;

n = size(A,1);
p = size(B,2);

Ts = 0.01;
sys_d = c2d(ss(A, B, C, D), Ts);

A = sys_d.a;
B = sys_d.b;

%init state
x0 = [pi/180;0];
x = x0;

%desired state
xd = [0;0];

%input
u0 = 0;
u = u0;

%augmented matrix construct
xa = [x; xd];
Aa = [A, zeros(n);zeros(n) eye(n)];
Ba = [B;zeros(n,p)];
Ca = [eye(n) -eye(n)];

R = 1;
Q = [1 0; 0 1];
S = [1 0; 0 1];

Sa = transpose(Ca)*S*Ca;
Qa = transpose(Ca)*Q*Ca;

F = F1_LQR_Gain(Aa, Ba, Qa, R, Sa);

steps = 100;

%states history
x_history = zeros(n, steps);
x_history(:,1) = x;

%input history
u_history = zeros(p,steps);
u_history(:,1) = u;

for i = 1:steps
    
    u = -F*xa;
    
    x = A*x + B*u;
    
    xa = [x;xd];
    
    x_history(:, i+1) = x;
    u_history(:, i+1) = u;
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
