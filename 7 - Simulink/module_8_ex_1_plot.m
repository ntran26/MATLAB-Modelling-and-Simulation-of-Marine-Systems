% Plotting for exercise 1

load M8_ex1_data.mat

t = out.data(:,1);
y = out.data(:,2);

plot(t,y);
grid on;
title("Sinewave Function");
xlabel("Time [s]");
ylabel("y [-]");
axis([0 10 -2 6]);

