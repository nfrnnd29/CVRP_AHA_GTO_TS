%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FunIndex: The index of function.                    %
% MaxIt: The maximum number of iterations.            %
% nPop: The size of hummingbird population.           %
% PopPos: The position of population.                 %
% PopFit: The fitness of population.                  %
% Dim: The dimensionality of prloblem.                %
% BestX: The best solution found so far.              %
% BestF: The best fitness corresponding to BestX.     %
% HisBestFit: History best fitness over iterations.   %
% Low: The low boundary of search space               %
% Up: The up boundary of search space.                %
% VisitTable: The visit table.                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%[Low,Up,Dim]=FunRange(FunIndex);
clc;
clear;
%%
tic;
nPop=500; %PopSize
MaxIt=1000;
%FunIndex=1;
Low = -1; %1
Up = 1000; %100
Dim = 32;
variables_no = Dim;
lb=ones(1,variables_no).*Low;
ub=ones(1,variables_no).*Up;


%%  Controlling parameter
p=0.03;
Beta=3;
w=0.8;

%% Dataset Augerat
instance = xlsread('A-n32-k5'); %P-n16-k8
node_demand = height(instance); %dikurangi depot harusnya 31
number_truck = 5; %8
truck_capacity = 100; %35

PopPos=zeros(nPop,Dim);
PopFit=zeros(1,nPop);
for i=1:nPop
    PopPos(i,:)=rand(1,Dim).*(Up-Low)+Low;
    [~,PopPos_idx]=sort(PopPos,2);
    [PopFit(i),~] = Obj_function_VRP_v3(instance,PopPos_idx(i,:),truck_capacity);
end


BestF=inf;
BestX=[];
Silverback=[];
Silverback_Score=inf;

for i=1:nPop
    if PopFit(i)<=BestF
        BestF=PopFit(i);
        BestX=PopPos(i,:);
    end
end

% Initialize visit table
HisBestFit=zeros(MaxIt,1);
VisitTable=zeros(nPop) ;
VisitTable(logical(eye(nPop)))=NaN;

for It=1:MaxIt
    DirectVector=zeros(nPop,Dim);% Direction vector/matrix

    for i=1:nPop
        r=rand;
        if r<1/3     % Diagonal flight
            RandDim=randperm(Dim);
            if Dim>=3
                RandNum=ceil(rand*(Dim-2)+1);
            else
                RandNum=ceil(rand*(Dim-1)+1);
            end
            DirectVector(i,RandDim(1:RandNum))=1;
        else
            if r>2/3  % Omnidirectional flight
                DirectVector(i,:)=1;
            else  % Axial flight
                RandNum=ceil(rand*Dim);
                DirectVector(i,RandNum)=1;
            end
        end

        if rand<0.5   % Guided foraging
            [MaxUnvisitedTime,TargetFoodIndex]=max(VisitTable(i,:));
            MUT_Index=find(VisitTable(i,:)==MaxUnvisitedTime);
            if length(MUT_Index)>1
                [~,Ind]= min(PopFit(MUT_Index));
                TargetFoodIndex=MUT_Index(Ind);
            end

            newPopPos=PopPos(TargetFoodIndex,:)+randn*DirectVector(i,:).*...
                (PopPos(i,:)-PopPos(TargetFoodIndex,:));


            Dim=length(newPopPos);
            S=(newPopPos>Up)+(newPopPos<Low);
            newPopPos=(rand(1,Dim).*(Up-Low)+Low).*S+newPopPos.*(~S);

            [~,newPopFit_idx]=sort(newPopPos,2);
            [newPopFit,~] = Obj_function_VRP_v3(instance,newPopFit_idx,truck_capacity);

            if newPopFit<PopFit(i)
                PopFit(i)=newPopFit;
                PopPos(i,:)=newPopPos;
                VisitTable(i,:)=VisitTable(i,:)+1;
                VisitTable(i,TargetFoodIndex)=0;
                VisitTable(:,i)=max(VisitTable,[],2)+1;
                VisitTable(i,i)=NaN;
            else
                VisitTable(i,:)=VisitTable(i,:)+1;
                VisitTable(i,TargetFoodIndex)=0;
            end
        else    % Territorial foraging
            newPopPos= PopPos(i,:)+randn*DirectVector(i,:).*PopPos(i,:);

            Dim=length(newPopPos);
            S=(newPopPos>Up)+(newPopPos<Low);
            newPopPos=(rand(1,Dim).*(Up-Low)+Low).*S+newPopPos.*(~S);

            [~,newPopFit_idx]=sort(newPopPos,2);
            [newPopFit,~] = Obj_function_VRP_v3(instance,newPopFit_idx,truck_capacity);

            if newPopFit<PopFit(i)
                PopFit(i)=newPopFit;
                PopPos(i,:)=newPopPos;
                VisitTable(i,:)=VisitTable(i,:)+1;
                VisitTable(:,i)=max(VisitTable,[],2)+1;
                VisitTable(i,i)=NaN;
            else
                VisitTable(i,:)=VisitTable(i,:)+1;
            end
        end
    end

    if mod(It,2*nPop)==0 % Migration foraging
        [~, MigrationIndex]=max(PopFit);
        PopPos(MigrationIndex,:) =rand(1,Dim).*(Up-Low)+Low;

        [~,PopFit_idx]=sort(PopPos,2);
        [PopFit(i),~] = Obj_function_VRP_v3(instance,PopFit_idx(i,:),truck_capacity);

        VisitTable(MigrationIndex,:)=VisitTable(MigrationIndex,:)+1;
        VisitTable(:,MigrationIndex)=max(VisitTable,[],2)+1;
        VisitTable(MigrationIndex,MigrationIndex)=NaN;
    end

    for i=1:nPop
        if PopFit(i)<BestF
            BestF=PopFit(i);
            BestX=PopPos(i,:);
        end
    end

    %% GTO
    GX = PopPos;
    X = PopPos;
    pop_size = nPop;
    a=(cos(2*rand)+1)*(1-It/MaxIt);
    C=a*(2*rand-1);

    for i=1:pop_size
        [~,X_idx]=sort(X,2);
        [Pop_Fit(i),~] = Obj_function_VRP_v3(instance,X_idx(i,:),truck_capacity);

        %Pop_Fit(i)=fobj(X(i,:));%#ok
        if Pop_Fit(i)<Silverback_Score
            Silverback_Score=Pop_Fit(i);
            Silverback=X(i,:);
        end
    end

    %% Exploration:

    for i=1:pop_size
        if rand<p
            GX(i,:) =(ub-lb)*rand+lb;
        else
            if rand>=0.5
                Z = unifrnd(-a,a,1,variables_no);
                H=Z.*X(i,:);
                GX(i,:)=(rand-a)*X(randi([1,pop_size]),:)+C.*H;
            else
                GX(i,:)=X(i,:)-C.*(C*(X(i,:)- GX(randi([1,pop_size]),:))+rand*(X(i,:)-GX(randi([1,pop_size]),:))); %ok ok

            end
        end
    end

    %GX = boundaryCheck(GX, lower_bound, upper_bound);

    for i=1:size(GX,1)
        FU=GX(i,:)>ub;
        FL=GX(i,:)<lb;
        GX(i,:)=(GX(i,:).*(~(FU+FL)))+ub.*FU+lb.*FL;
    end

    % Group formation operation
    for i=1:pop_size
        %New_Fit= fobj(GX(i,:));
        [~,GX_idx]=sort(GX,2);
        [New_Fit,~] = Obj_function_VRP_v3(instance,GX_idx(i,:),truck_capacity);
        if New_Fit<Pop_Fit(i)
            Pop_Fit(i)=New_Fit;
            X(i,:)=GX(i,:);
        end
        if New_Fit<Silverback_Score
            Silverback_Score=New_Fit;
            Silverback=GX(i,:);
        end
    end

    %% Exploitation:
    for i=1:pop_size
        if a>=w
            g=2^C;
            delta= (abs(mean(GX)).^g).^(1/g);
            GX(i,:)=C*delta.*(X(i,:)-Silverback)+X(i,:);
        else

            if rand>=0.5
                h=randn(1,variables_no);
            else
                h=randn(1,1);
            end
            r1=rand;
            GX(i,:)= Silverback-(Silverback*(2*r1-1)-X(i,:)*(2*r1-1)).*(Beta*h);

        end
    end

    %GX = boundaryCheck(GX, lower_bound, upper_bound);

    for i=1:size(GX,1)
        FU=GX(i,:)>ub;
        FL=GX(i,:)<lb;
        GX(i,:)=(GX(i,:).*(~(FU+FL)))+ub.*FU+lb.*FL;
    end

    % Group formation operation
    for i=1:pop_size
        %New_Fit= fobj(GX(i,:));

        [~,GX_idx]=sort(GX,2);
        [New_Fit,~] = Obj_function_VRP_v3(instance,GX_idx(i,:),truck_capacity);

        if New_Fit<Pop_Fit(i)
            Pop_Fit(i)=New_Fit;
            X(i,:)=GX(i,:);
        end
        if New_Fit<Silverback_Score
            Silverback_Score=New_Fit;
            Silverback=GX(i,:);
        end
    end

    BestF=Silverback_Score;
    BestX=Silverback;

    initial = randperm(Dim);
    p = randperm(Dim,2);
    k = randperm(3,1);
    [~, BestX_idx]=sort(BestX,2);
    switch k
        case 1
            [init_one] = flap_mut(BestX_idx,p);
            BestX_idx = init_one;
            BestX = BestX(BestX_idx);
        case 2
            [initial] = interchange_mut(BestX_idx,p);
            BestX_idx = initial;
            BestX = BestX(BestX_idx);
        case 3
            [initial_one] = slide_mut(BestX_idx,p);
            BestX_idx = initial_one;
            BestX = BestX(BestX_idx);
    end



    %     %% Local Search
    %     if It > MaxIt/2
    %         improved = 1; %True
    %         [~, BestX_idx]=sort(BestX,2);
    %         g_best_found_route = BestX_idx;
    %         g_best_found_route_cost = BestF;
    %
    %         while improved == 1
    %             improved = 0;
    %             for i = 1: length(g_best_found_route)-1
    %                 for k = i+1 : length(g_best_found_route)-1
    %                     first_new_route = g_best_found_route(1:i);
    %                     reversed_route = fliplr(g_best_found_route(i+1:k));
    %                     end_new_route =  g_best_found_route(k+1:end);
    %                     new_route = [first_new_route reversed_route end_new_route];
    %                     %[~,new_route_idx] = sort(new_route,2);
    %                     [new_route_cost, ~] = Obj_function_VRP_v3(instance,new_route,truck_capacity);
    %                     if new_route_cost < g_best_found_route_cost %;
    %                         g_best_found_route_cost = new_route_cost;
    %                         g_best_found_route = new_route;
    %                         if g_best_found_route_cost < BestF
    %                             BestF = g_best_found_route_cost;   %Updating the fitness function value of the best solution
    %                             %Alpha_pos = g_best_found_route_element;     %Updating the best solution
    %                             BestX_idx = g_best_found_route; %Determining the best track solution
    %                             BestX = BestX(BestX_idx);
    %                         end
    %                         improved = 1;
    %                         break
    %                     end
    %                 end
    %                 if improved == 1
    %                     break
    %                 end
    %             end
    %         end
    %     end

    HisBestFit(It)=BestF;
    disp(['Iteration ' num2str(It) ': Best fitness =' num2str(HisBestFit(It))]);
end

%% Plot
plot(1:MaxIt,HisBestFit);
xlabel('Iteration');
ylabel('Best fitness value');

%display(['FunIndex=', num2str(FunIndex)]);
toc
display(['The best Position is: ', num2str(BestX)]);
[~,BestX_idx]=sort(BestX,2);
display(['The best Route is: ', num2str(BestX_idx)]);
display(['The best fitness is: ', num2str(BestF)]);

[~, route_sequence] = Obj_function_VRP_v3(instance,BestX_idx,truck_capacity);
for g = 1:length(route_sequence)
    disp(['Vehicle Route ' num2str(g) ': ' num2str(cell2mat(route_sequence{1,g}))])
end

%% Tabu Search

%% Problem Definition

model = CreateModel();      % Create TSP Model

ActionList=CreatePermActionList(Dim);    % Action List

nAction=numel(ActionList);              % Number of Actions


%% Tabu Search Parameters

%MaxIt=Max_iter;                      % Maximum Number of Iterations
tabu_iter = 1000; %100

TL=round(0.5*nAction); %round(0.5*nAction);      % Tabu Length, 0.25 bisa cepet ketemu

%% Initialization

% Create Empty Individual Structure
empty_individual.Position=[];
empty_individual.Cost=[];

% Array to Hold Best Costs
BestCost=zeros(tabu_iter,1);%zeros(Max_iter,1);
%BestSchedule =zeros(MaxIt,2*n_job);
BestSchedule = {}';

% Initialize Action Tabu Counters
TC=zeros(nAction,1);

%% Initialization Tabu Search

% Create Initial Solution
sol=empty_individual;
sol.Position=BestX_idx;
sol.Cost=BestF;

% Initialize Best Solution Ever Found
BestSol=sol;

%% Tabu Search Main Loop

for it=1:tabu_iter %Max_iter

    bestnewsol.Cost=inf;

    % Apply Actions
    for i=1:nAction
        if TC(i)==0
            newsol.Position=DoAction(sol.Position,ActionList{i});
            [new_route_costs, ~] = Obj_function_VRP_v3(instance,newsol.Position,truck_capacity);
            newsol.Cost=new_route_costs;
            newsol.ActionIndex=i;

            if newsol.Cost<=bestnewsol.Cost
                bestnewsol=newsol;
            end
        end
    end

    % Update Current Solution
    sol=bestnewsol;

    % Update Tabu List
    for i=1:nAction
        if i==bestnewsol.ActionIndex
            TC(i)=TL;               % Add To Tabu List
        else
            TC(i)=max(TC(i)-1,0);   % Reduce Tabu Counter, max
        end
    end

    % Update Best Solution Ever Found
    if sol.Cost<=BestSol.Cost
        BestSol=sol;
    end

    % Save Best Cost Ever Found
    BestCost(it)=BestSol.Cost;
    BestRoute(it) = mat2cell(BestSol.Position,1);

    disp(['Iteration of TS ' num2str(it) ': Best Cost = ' num2str(BestCost(it))]); %' Best Schedule =' cell2mat(BestSchedule(it))
end