
Kp = 10;
Kd = 10;
ki = 0;

num = [0,1231.468, 1.6];
den = [2342269.5,0, 87.808];
sys = tf(num, den)


error = 1;

c = pid(Kp,ki,Kd);

T = feedback(sys*c,error)
subplot(3, 1, 1)
step(sys)
subplot(3, 1, 2)
step(T)
% tfinal = 10000;
% t = (0:0.1:tfinal);
% r = t.^0;
% lsim(T,r,t)


% t = (0:0.1:10)';
% tfinal  = t.^2;
% T = feedback(sys*c,error);
% step(T, tfinal)
subplot(3, 1, 3)
rlocus(sys)


