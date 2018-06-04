num = [0, 0, 1.5]
den = [0.4, 0, 0]
g = tf(num, den)
subplot(2, 2, 1)
step(g)
kp = 1
ki = 0
kd = 1

c = pid(kp, ki, kd)

error = 1

t = feedback(g*c, error)+1.568
subplot(2, 2, 2)
step(t)