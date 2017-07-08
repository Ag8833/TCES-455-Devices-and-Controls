%Resistor = 9600k ohms
%Capacitor = 100 uF
%Tau = 1.05 - .1 = .95 seconds
%TF = 1/(.95s + 1)

Ts = .1;

plot(0:0.1:5,eo_act(101:151),'b*-');
hold on
plot(0:0.1:5,eo_fp(101:151),'r--');
plot(0:0.1:5,eo_bb(101:151),'k--');
xlabel('time (seconds)')
ylabel('voltage (Volts)')
title('RC Circuit Free Response')
legend('experiment','model (first principles)','model (blackbox)')