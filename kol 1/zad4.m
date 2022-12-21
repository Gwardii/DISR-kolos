syms q dq ddq [2,1] real

l = sqrt(2)/2;
m1 = 20;
J_C1 = diag([1,0.1,1]);
m2 = 10;
J_C2 = diag([0.2,2,2]);
F = [100;200;0];
g = [0;-10;0];
q_ = [1;pi/4];
dq_ = [1, 2];
tau = [370; -50];

R_z = @(phi)[utils.omega(phi),zeros(2,1);0,0,1];

r1 = [0;q1;0];
r2 = r1;
r_tcp = r1+R_z(q2)*[l;0;0]
R_20 = R_z(q2);

J_tcp = jacobian(r_tcp,q)
J_C2_0 = R_20*J_C2*R_20'

dr1 = jacobian(r1,q)*dq
dr2 = dr1
omega_2 = [0;0;dq2]

Ek = 0.5*(m1*dr1'*dr1+m2*dr2'*dr2 + omega_2'*J_C2_0*omega_2)

Ep = -g'*(m1*r1 + m2*r2)

L = Ek-Ep;
L_dq = jacobian(L,dq)
L_q = jacobian(L,q)

Q = J_tcp' * F

dL_dq = [30*ddq1; 2*ddq2]

Q_ = subs(Q,q,q_)

solution = solve(dL_dq - L_q' == Q_+tau,ddq)

