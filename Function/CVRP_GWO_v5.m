clear all 
clc

SearchAgents_no=1000; % Number of search agents

%Function_name='F10'; % Name of the test function that can be from F1 to F23 (Table 1,2,3 in the paper)

Max_iter=5; % Maximum numbef of iterations

% Load details of the selected benchmark function
%[lb,ub,dim,fobj]=Get_Functions_details(Function_name);

% fobj = @F1;

lb=1000;
ub=1000;
dim=33;

%% Dataset Augerat
instance = xlsread('A-n33-k6'); %P-n16-k8 , A-n33-k5 = 661, A-n33-k6, A-n32-k5
node_demand = height(instance); %dikurangi depot harusnya 31
number_truck = 6;
truck_capacity = 100;

%[Best_score,Best_pos,GWO_cg_curve]=GWO(SearchAgents_no,Max_iteration,lb,ub,dim,fobj);

%Alpha_score,Alpha_pos,Convergence_curve

% initialize alpha, beta, and delta_pos
Alpha_pos=zeros(1,dim);
Alpha_score=inf; %change this to -inf for maximization problems

Beta_pos=zeros(1,dim);
Beta_score=inf; %change this to -inf for maximization problems

Delta_pos=zeros(1,dim);
Delta_score=inf; %change this to -inf for maximization problems

BestFitIter = NaN(Max_iter,1);

%Initialize the positions of search agents
%Positions=initialization(SearchAgents_no,dim,ub,lb);

Boundary_no= size(ub,2); % numnber of boundaries

% If the boundaries of all variables are equal and user enter a signle
% number for both ub and lb
if Boundary_no==1
    Positions=rand(SearchAgents_no,dim);%.*(ub-lb)+lb;
end

% If each variable has a different lb and ub
if Boundary_no>1
    for i=1:dim
        ub_i=ub(i);
        lb_i=lb(i);
        Positions(:,i)=rand(SearchAgents_no,1);%.*(ub_i-lb_i)+lb_i;
    end
end

Positions;

[~,P_idx] = sort(Positions,2);

Convergence_curve=zeros(1,Max_iter);

l=0;% Loop counter

% Main loop
while l<Max_iter
    for i=1:size(Positions,1)  
        
       % Return back the search agents that go beyond the boundaries of the search space
        %Flag4ub=Positions(i,:)>ub;
        %Flag4lb=Positions(i,:)<lb;
        %Positions(i,:)=(Positions(i,:).*(~(Flag4ub+Flag4lb)))+ub.*Flag4ub+lb.*Flag4lb;    
        [~,P_idx] = sort(Positions,2);
        
        % Calculate objective function for each search agent
        %fitness=fobj(Positions(i,:));
        [fitness,~] = Obj_function_VRP_v3(instance,(P_idx(i,:)),truck_capacity);
        
        % Update Alpha, Beta, and Delta
        if fitness<Alpha_score 
            Alpha_score=fitness; % Update alpha
            Alpha_pos=Positions(i,:);
            %[~,P_idx] = sort(Alpha_pos,2);
        end
        
        if fitness>Alpha_score && fitness<Beta_score 
            Beta_score=fitness; % Update beta
            Beta_pos=Positions(i,:);
        end
        
        if fitness>Alpha_score && fitness>Beta_score && fitness<Delta_score 
            Delta_score=fitness; % Update delta
            Delta_pos=Positions(i,:);
        end
    end
    
    
    a=2-l*((2)/Max_iter); % a decreases linearly fron 2 to 0
    
    % Update the Position of search agents including omegas
    for i=1:size(Positions,1)
        for j=1:size(Positions,2)     
                       
            r1=rand(); % r1 is a random number in [0,1]
            r2=rand(); % r2 is a random number in [0,1]
            
            A1=2*a*r1-a; % Equation (3.3)
            C1=2*r2; % Equation (3.4)
            
            D_alpha=abs(C1*Alpha_pos(j)-Positions(i,j)); % Equation (3.5)-part 1
            X1=Alpha_pos(j)-A1*D_alpha; % Equation (3.6)-part 1
                       
            r1=rand();
            r2=rand();
            
            A2=2*a*r1-a; % Equation (3.3)
            C2=2*r2; % Equation (3.4)
            
            D_beta=abs(C2*Beta_pos(j)-Positions(i,j)); % Equation (3.5)-part 2
            X2=Beta_pos(j)-A2*D_beta; % Equation (3.6)-part 2       
            
            r1=rand();
            r2=rand(); 
            
            A3=2*a*r1-a; % Equation (3.3)
            C3=2*r2; % Equation (3.4)
            
            D_delta=abs(C3*Delta_pos(j)-Positions(i,j)); % Equation (3.5)-part 3
            X3=Delta_pos(j)-A3*D_delta; % Equation (3.5)-part 3             
            
            Positions(i,j)=(X1+X2+X3)/3;% Equation (3.7)
            [~,P_idx] = sort(Positions,2);
            
        end
    end

%     %% Local Search
%     improved = 1; %True
%     [~, alpha_pos_idx]=sort(Alpha_pos,2);
%     g_best_found_route = alpha_pos_idx;
%     g_best_found_route_cost = Alpha_score;
% 
%     while improved == 1
%         improved = 0;
%         for i = 1: length(g_best_found_route)-1
%             for k = i+1 : length(g_best_found_route)-1
%                 first_new_route = g_best_found_route(1:i);
%                 reversed_route = fliplr(g_best_found_route(i+1:k));
%                 end_new_route =  g_best_found_route(k+1:end);
%                 new_route = [first_new_route reversed_route end_new_route];
%                 %[~,new_route_idx] = sort(new_route,2);
%                 [new_route_cost, ~] = Obj_function_VRP_v3(instance,new_route,truck_capacity);
%                 if new_route_cost < g_best_found_route_cost %;
%                     g_best_found_route_cost = new_route_cost;
%                     g_best_found_route = new_route;
%                     if g_best_found_route_cost < Alpha_score
%                         Alpha_score = g_best_found_route_cost;   %Updating the fitness function value of the best solution
%                         %Alpha_pos = g_best_found_route_element;     %Updating the best solution
%                         alpha_pos_idx = g_best_found_route; %Determining the best track solution
%                         Alpha_pos = Alpha_pos(alpha_pos_idx);
%                     end
%                     improved = 1;
%                     break
%                 end
%             end
%             if improved == 1
%                 break
%             end
%         end
%     end

% %% Local Search
%     improved = 1; %True
%     [~, alpha_pos_idx]=sort(Alpha_pos,2);
%     g_best_found_route = alpha_pos_idx;
%     g_best_found_route_cost = Alpha_score;
% 
%     while improved == 1
%         improved = 0;
%         for i = 1:Max_iter
%             new_route = mutation_6(g_best_found_route,dim);
%             [new_route_cost, ~] = Obj_function_VRP_v3(instance,new_route,truck_capacity);
%             if new_route_cost < g_best_found_route_cost %;
%                 g_best_found_route_cost = new_route_cost;
%                 g_best_found_route = new_route;
%                 if g_best_found_route_cost < Alpha_score
%                     Alpha_score = g_best_found_route_cost;   %Updating the fitness function value of the best solution
%                     alpha_pos_idx = g_best_found_route; %Determining the best track solution
%                     Alpha_pos = Alpha_pos(alpha_pos_idx);
%                 end
%                 improved = 1;
%                 break
%             end
% %             if improved == 1
% %                 break
% %             end
%         end
%     end
    
    l=l+1;
    BestFitIter(l+1) = Alpha_score;
    disp(['Iteration ' num2str(l) ': Best fitness =' num2str(BestFitIter(l+1))]);
    %Alpha_score
    Convergence_curve(l)=Alpha_score;
end

%% Solution
display(['The best solution obtained by GWO is : ', num2str(Alpha_pos)]);
[~,alpha_pos_idx] = sort(Alpha_pos,2);
%display(['The best route obtained by GWO is : ', num2str(alpha_pos_idx)]);
%display(['The best optimal value of the objective funciton found by GWO is : ', num2str(Alpha_score)]);
[~, route_sequence] = Obj_function_VRP_v3(instance,alpha_pos_idx,truck_capacity);

for g = 1:length(route_sequence)
    disp(['Vehicle Route ' num2str(g) ': ' num2str(cell2mat(route_sequence{1,g}))]);
end

%% Plot
plot(0:Max_iter,BestFitIter);
xlabel('Iteration');
ylabel('Best fitness value');