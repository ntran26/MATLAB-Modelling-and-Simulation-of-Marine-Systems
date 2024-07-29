% Transfer function
sys1 = tf([5],[1 3 7])

% State space model
sys2 = ss(sys1)

% Plot step response
step(sys1)
step(sys2)