function [F] = F1_LQR_Gain(A,B,Q,R,S)

%calculate system matrix dim
n = size(A,1);
%calculatre input matrix dim
p = size(B,2);

%cost weight of system end value 1/2{x(N)P0x(N)}
P0 = S;
%max number of iterations 
max_iter = 200;
%initiate  matrix P used to save P[k]
P = zeros(n, n*max_iter);
%initiate first position of P
P(:,1:n) = P0;
%define P[k-1], set init value as P0, i.e k = 1
P_k_min_1 = P0;
%define threschold of steady state error
tol = 1e-3;
%initiate system error
diff = Inf;
%initiate system feedback gain F[N-k]
F_N_min_k = Inf;

k = 1;

while diff > tol
    F_N_min_k_pre = F_N_min_k;
    %cal F[N-k]
    F_N_min_k = inv(R+B'*P_k_min_1*B)*B'*P_k_min_1*A
    %cal P[k]
    P_k = (A-B*F_N_min_k)'*P_k_min_1*(A-B*F_N_min_k)+(F_N_min_k)'*R*(F_N_min_k)+Q;
    %save P[k] into P in a correct sequence
    P(:, n*k-n+1:n*k) = P_k;
    
    P_k_min_1 = P_k;
    
    %cal diff between two adjacent gains
    diff = abs(max(F_N_min_k - F_N_min_k_pre));
    
    k = k+1;
    
    if (k > max_iter)
        error('Maximun Number of Iteration Exceeded');
    end
end

fprintf('No. of Iteration is %d \n', k);

F = F_N_min_k;
end