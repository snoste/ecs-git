function test()

format short;
clc;
clear all;

%% Response time analysis -- preemptive case

%% period, deadline, wcet and prioriy

%PU1                  Task    Inchron Priorities
 T = [5, 10, .1, 1;   % Ts    4
      10, 1, 2, 2;    % T1    3
      15, 1, 2, 3;    % T2    2
      20, 5, 3, 4;    % T3    1
    ];

number_of_tasks = length(T(:,1));

for task = 1:number_of_tasks
    R(task) = 0;      
    while(1)
        R_pre = R(task);
        HP_waiting = 0;
        for i = 1:number_of_tasks
            if (i ~= task) && (T(i,4)<T(task,4))
                HP_waiting = HP_waiting + ceil(R(task)/T(i,1))*T(i,3);           
            end             
        end    
        R(task) = T(task,3) + HP_waiting;
%         %% already missed deadline
%         if R(task) > T(task,2)
%             R(task) = -1;
%             break;
%         end
        %% termination condition
        if (R(task) == R_pre)
            break;            
        end
    end
end



R

