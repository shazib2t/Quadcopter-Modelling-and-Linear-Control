%Thrust or Force VS Angular Speed curve
w = 0:0.1: 2;
f = 1.5*w.^2;
title('Rotor Physics')
xlabel('Angular Speed(w)')
ylabel('Force(thrust) and Moment')

plot(w, f)
hold on
%Moment or drag VS Angular Speed curve
w = 0:0.1: 2;
m = 1.9*w.^2;
title('Rotor Dynamics')
xlabel('Angular Speed(w)')
ylabel('Force(thrust) and Moment')
plot(w, m)
