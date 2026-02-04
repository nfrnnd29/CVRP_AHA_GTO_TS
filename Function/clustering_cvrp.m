%Partition Data into Two Clusters
%Randomly generate the sample data.

clc
clear

rng default; % For reproducibility
% X = [randn(100,2)*0.75+ones(100,2);
%     randn(100,2)*0.5-ones(100,2)];

instance = xlsread('A-n36-k5');

X = instance(:,2:3);

depot = instance(1,:);

X(1,:) = [];

% figure;
% plot(X(:,1),X(:,2),'.');
% hold on
% plot(depot(:,2),depot(:,3),'kx','MarkerSize',15,'LineWidth',3) 
% title 'A-n33-k6 Dataset Scatter Plot';

%Cluster
%There appears to be two clusters in the data.

%Partition the data into two clusters, and choose the best arrangement out of five initializations. Display the final output.
rng('default'); % For reproducibility
[idx, C, SUMD, D, MIDX, INFO] = kmedoids(X,5,'distance','seuclidean','replicates',3); %KALO GANTI DATA, GANTI BAGIAN INI JUGA
%By default, the software initializes the replicates separately using k-means++.

%Plot the clusters and the cluster centroids.
% figure;
% plot(X(idx==1,1),X(idx==1,2),'r.','MarkerSize',12);
% hold on
% plot(X(idx==2,1),X(idx==2,2),'b.','MarkerSize',12);
% hold on
% plot(X(idx==3,1),X(idx==3,2),'y.','MarkerSize',12);
% hold on
% plot(X(idx==4,1),X(idx==4,2),'m.','MarkerSize',12);
% hold on
% plot(X(idx==5,1),X(idx==5,2),'g.','MarkerSize',12);
% % hold on
% % plot(X(idx==6,1),X(idx==6,2),'k.','MarkerSize',12)
% plot(C(:,1),C(:,2),'kx',...
%      'MarkerSize',15,'LineWidth',3) 
% % legend('Cluster 1','Cluster 2','Cluster 3','Cluster 4','Cluster 5','Cluster 6','Centroids',...
% %        'Location','eastoutside') %legend('Location','NW')
% legend('Cluster 1','Cluster 2','Cluster 3','Cluster 4','Cluster 5','Centroids',...
%        'Location','eastoutside'); %legend('Location','NW')
% title 'Cluster Assignments and Centroids'
% hold off

red_cluster = [X(idx==1,1) X(idx==1,2)];
blue_cluster =[X(idx==2,1) X(idx==2,2)];
yellow_cluster = [X(idx==3,1) X(idx==3,2)];
magenta_cluster = [X(idx==4,1) X(idx==4,2)];
green_cluster = [X(idx==5,1) X(idx==5,2)];
% black_cluster = [X(idx==6,1) X(idx==6,2)] %KALO GANTI DATA, GANTI BAGIAN INI JUGA

%after_cluster = [red_cluster; blue_cluster; yellow_cluster; magenta_cluster; green_cluster; black_cluster] 
after_cluster = [red_cluster; blue_cluster; yellow_cluster; magenta_cluster; green_cluster]; %KALO GANTI DATA, GANTI BAGIAN INI JUGA

after_cluster_node = {};
for i = 1:length(after_cluster)
    RowIdx = find(ismember(X, after_cluster(i,:),'rows'));
    after_cluster_node{end+1} = RowIdx+1;
end

after_cluster_node = cell2mat(after_cluster_node);

after_cluster_node = [1 after_cluster_node];

truck_capacity = 100;
[fitness_cluster,route_sequence] = Obj_function_VRP_v3(instance,after_cluster_node,truck_capacity);
fitness_cluster

for g = 1:length(route_sequence)
    disp(['Vehicle Route ' num2str(g) ': ' num2str(cell2mat(route_sequence{1,g}))]);
end

number_pop = 1000;%coba ganti2 iki sisan
for n = 1:number_pop
    Pop(n,:) = randperm(length(instance));
end

for i = 1:number_pop
    cost(i,:) = Obj_function_VRP_v3(instance,(Pop(i,:)),truck_capacity);
end

for n = 1:10/100*number_pop%number_pop/2
    Pop(n,:) = after_cluster_node;
    cost(n,:) = fitness_cluster;
end

%Genetic Algorithm

%% Parameters
N = number_pop; % Population Size
G = length(instance); % Genome Size
PerMut = 1; % Probability of Mutation
S = 2; % Tournament Size
F = cost;
%[~,Pop] = sort(rand(N,G),2); % Create Initial Population

LocalIter = 3000; %coba ganti2 iki sisan
MaxIter = 1000; %coba ganti2 iki sisan

for Gen = 1:MaxIter % Number of Generations
    %% Fitness
    %F = var(diff(Pop,[],2),[],2); % Measure Fitness
    %% Print Stats
    %fprintf('Gen: %d Mean Fitness: %d Best Fitness: %d\n', Gen, round(mean(F)), round(max(F)))

    %% Selection (Tournament)
    T = round(rand(2*N,S)*(N-1)+1); % Tournaments
    %[~,idx] = max(F(T),[],2); % Index to Determine Winners
    [~,idx] = min(F(T),[],2);
    W = T(sub2ind(size(T),(1:2*N)',idx)); % Winners

    %% Crossover (Single!Point Preservation)
    Pop2 = Pop(W(1:2:end),:); % Assemble Pop2 Winners 1
    P2A = Pop(W(2:2:end),:); % Assemble Pop2 Winners 2
    Lidx = sub2ind(size(Pop),[1:N]',round(rand(N,1)*(G-1)+1)); % ...
    %Select Point
    vLidx = P2A(Lidx)*ones(1,G); % Value of Point in Winners 2
    [r,c] = find(Pop2 == vLidx); % Location of Values in ... Winners 1
    [~,Ord] = sort(r); % Sort Linear Indices
    r = r(Ord); c = c(Ord); % Re-order Linear Indices
    Lidx2 = sub2ind(size(Pop),r,c); % Convert to Single Index
    Pop2(Lidx2) = Pop2(Lidx); % Crossover Part 1
    Pop2(Lidx) = P2A(Lidx); % Validate Genomes

    %% Mutation (Permutation)
    idx = rand(N,1)<PerMut; % Individuals to Mutate
    Loc1 = sub2ind(size(Pop2),1:N,round(rand(1,N)*(G-1)+1)); % Index ... Swap 1
    Loc2 = sub2ind(size(Pop2),1:N,round(rand(1,N)*(G-1)+1)); % Index ... Swap 2
    Loc2(idx == 0) = Loc1(idx==0); % Probabalistically Remove ... Swaps
    [Pop2(Loc1),Pop2(Loc2)] = deal(Pop2(Loc2), Pop2(Loc1)); % Perform ... Exchange

    [f_pbest,ind_f] = min(cost);
    BestFitPopIter(Gen) = f_pbest;
    pbest(Gen,:) = Pop(ind_f,:);
    
    if Gen == 1
        [f_gbest,ind_f] = min(BestFitPopIter);
        gbest = pbest(ind_f,:);
        BestFitIter(Gen) = f_gbest;
    elseif Gen > 1
        if BestFitIter(Gen-1) < f_gbest
            f_gbest = BestFitIter(Gen-1);
            gbest = pbest(ind_f,:);
            BestFitIter(Gen) = f_gbest;
        elseif BestFitIter(Gen-1) > f_gbest
            [f_gbest,ind_f] = min(BestFitPopIter);
            gbest = pbest(ind_f,:);
            BestFitIter(Gen) = f_gbest;
        end
    end

    for j = 1:LocalIter
        %Gen
        %j
        new_route = mutation_6(gbest,G);
        [new_route_cost, ~] = Obj_function_VRP_v3(instance,new_route,truck_capacity);
        %BestFitIter(Gen)
        %f_gbest
        if new_route_cost <= f_gbest %BestFitIter(Gen)
            f_gbest = new_route_cost;
            gbest = new_route;
            %if BestFitIter(Gen) < f_gbest
            %    f_gbest = BestFitIter(Gen)
            %end
        end
    end
    
    new_route_cost = f_gbest;
    BestFitIter(Gen) = f_gbest;

    disp(['Iteration ' num2str(Gen) ': Best fitness =' num2str(BestFitIter(Gen)) ' Best Route =' num2str(gbest)]); %BestFitIter(Gen)

    %% Reset Population
    Pop = Pop2;

    for z = 1:(20/100)*number_pop %coba ganti2 angka 20 nya, iki ben iso disimpen hasil gbest e dipake di iterasi berikutnya berapa persen
        min_initial = gbest;
        min_value = f_gbest;
        cost(z,:) = min_value;
        Pop(z,:) = min_initial;
    end
    
end

[~,gbest_route_sequence] = Obj_function_VRP_v3(instance,gbest,truck_capacity);
for g = 1:length(gbest_route_sequence)
    disp(['Vehicle Route ' num2str(g) ': ' num2str(cell2mat(gbest_route_sequence{1,g}))]);
end

Plot
plot(1:MaxIter,BestFitIter);
xlabel('Iteration');
ylabel('Best fitness');
