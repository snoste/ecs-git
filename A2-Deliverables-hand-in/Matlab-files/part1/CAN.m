function test()

format short;
clc;
clear all;

%% Response time analysis -- non-preemptive case

%% period, deadline, wcet and prioriy
 T = [5, 2, 1, 1;   % ms
      10, 15, 1, 2; % m1
      20, 30, 1, 4; % m3
      10, 10, 1, 3; % m8
      ];

tau_res = 0.00001;
number_of_tasks = length(T(:,1));

for task = 1:number_of_tasks
  blocking_period(task) = 0;
  for i = 1:number_of_tasks
      if (i ~= task) && (T(i,4)>T(task,4)) && blocking_period(task) < T(i,3)
        blocking_period(task) = T(i,3);
%         blocking_period(task) = 0;
      end             
  end  
end

blocking_period

for task = 1:number_of_tasks
    busy_period(task) = T(task,3);     
    while(1)
        busy_period_pre = busy_period(task);
        busy_waiting = 0;
        for i = 1:number_of_tasks
            if (T(i,4)<=T(task,4))
                busy_waiting = busy_waiting + ceil(busy_period(task)/T(i,1))*T(i,3);           
            end             
        end    
        busy_period(task) = blocking_period(task) + busy_waiting;
        if (busy_period(task) == busy_period_pre)
            break;            
        end
    end
end

busy_period

for task = 1:number_of_tasks
    if busy_period(task) <= T(task,1)
       R(task) =  busy_period(task);
    else 
        Qm = ceil(busy_period(task)/T(task,1));
        for i=1:Qm
            Response_Time(i) = blocking_period(task) + (i-1)*T(task,3);
            while(1)
                R_pre = Response_Time(i);
                HP_waiting = 0;
                for j = 1:number_of_tasks
                    if (T(j,4)<T(task,4))
                        HP_waiting = HP_waiting + ceil((Response_Time(i)+tau_res)/T(j,1))*T(j,3);           
                    end 
                end
                Response_Time(i) = blocking_period(task) + (i-1)*T(task,3) + HP_waiting;
                if (Response_Time(i) == R_pre)
                    break;            
                end
            end 
             Response_Time(i) = Response_Time(i) - (i-1)*T(task,1) + T(task,3);                  
        end 
        Response_Time
        R(task) = 0;
        for i=1:Qm
            if Response_Time(i)>R(task)
                R(task) = Response_Time(i);
            end
        end

    end    
end

 R