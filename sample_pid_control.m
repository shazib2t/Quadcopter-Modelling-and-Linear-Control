
clear all;
clc;

G = tf([1,1],[1, 1, 0]);

step(G);
h = 1;

step(feedback(G,h));
%%
kp = 1;
kd= 1;
ki = 0;
c= pid(kp,ki,kd)

T = feedback(G*c,h)
step(T)
