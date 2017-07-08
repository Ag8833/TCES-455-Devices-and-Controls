Ra = .0325;     %Ohms
La = .000014;   %H
Kv = .0631;     %Vs
Jmotor = .0117; %kg-m^2
Bm = 500*10^-6; %Nms
Jload = .6883;  %kg-m^2
KL = 12*10^-6;  %Nms^2
va = 24;        %V
iaic = 0.0;     %A
wmic = 0.0;     %rad/s

Jtot = Jmotor + Jload;

t = simout(:,1);
iaic = simout(:,2);
wmic = simout(:,3);

plot(t,iaic,'b');
hold on;
plot(t,wmic,'g');
xlabel('Time');
legend('iaic','wmic');