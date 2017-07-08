s = tf('s');                        %Function to set s as a transfer function variable
Plant = 1/(s^2+20*s+20);            %Plant function
K1 = 0:0.1:10;
K2 = 0:0.1:10;
for i = 1:length(K1)
    L = K1(i) * Plant;              %K * Plant to get the top line of the block diagram
    Cloop1 = feedback(L,1);         %Close this in a negative feedback loop
    Y1 = step(Cloop1);              %Make the output be a step response of the feedback loop
    SSY1(i) = Y1(end);              %Save the output into an array
end
for i = 1:length(K2)
    Cloop2 = feedback(Plant,K2(i)); %Make this a feedback loop by connecting Plant into K
    Y2 = step(Cloop2);              %Make the output be a step response of the feedback loop
    SSY2(i) = Y2(end);              %Save the output into an array
end
for i = 1:length(K1)
    if SSY1(i) == SSY2(i)
        K1_Intersection = K1(i)
    end
end    
plot(K1, SSY1, 'g')
hold on                             %Allows for 2 graphs simultaneously.
plot(K2, SSY2, 'b')
xlabel('K')
ylabel('Steady State Value')
legend('Step','Disturbance')

%The value of K is 1 where the 2 graphs cross.