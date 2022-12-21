t0 = 0;
tk = 10;
Ta = 2;
l = 1.6;
rho = 0.5;
r_p = [1.8;1];
r_b = r_p-[rho;0]
r_a = r_p + [rho;0]
t = 6.8;

p = Ta/(tk-t0)
tau = (t-t0)/(tk-t0)
s = (tau-p/2)/(1-p)

r = r_p + utils.omega(pi*s)*(r_a-r_p)

q2 = asin(r(2)/l)
q1 = r(1) - l*cos(q2)