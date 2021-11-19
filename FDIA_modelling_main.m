%close all
clear
addpath(genpath('matpower7.0/')) %add all subfolders
casename = 'case30.m';

%% simulate reality/environment
if ~(exist('measure')==1)
    disp("simulate grid enviroment and meters by power flow")
    mpc = loadcase(casename);
    %simulate truth by power flow, and SCADA meters with noise
    noise_level = 0.001; %std of gaussian noise to be added on meters
    pf_results = runpf(mpc);
    ref = find(mpc.bus(:,2)==3);
    [measure,idx,sigma,GT,V_GT] = simulate_SCADA_meters(pf_results, noise_level);
    %% add bad data (traditional)
    if_bad = 0;
    if if_bad==1
        %add one bad data
        measure.Pinj(3) = sign(measure.Pinj(3))*(1+abs(measure.Pinj(3)));
    end
end
%% operator's previous SE settings (attackers can steal previous state estimates at t-1)
%control parameters
K_diff_topo = 1; %create the difference in topology between t-1 and t
K_diff_para = 2; %create number of outdated line parameter t-1 and t
Mag_prev_para = 0.02;
prev_lf = rand(1)*0.05+0.95; %randomly sample a previous lf between 100%+/-2.5%
%
mpc_prev = mpc;
shuffled_br_ids_topo = randperm(length(mpc_prev.branch));
%make topolgy different
mpc_prev.branch = get_inaccurate_topology(mpc_prev.branch,K_diff_topo,shuffled_br_ids_topo);
%make network paramter different (sparse)
shuffled_br_ids_para = randperm(length(mpc_prev.branch));
[mpc_prev.branch, ~] = get_inaccurate_para_sparse(mpc_prev.branch,K_diff_para,Mag_prev_para,shuffled_br_ids_para);
%load factor different
mpc_prev.bus(:,3:4) = mpc_prev.bus(:,3:4)*prev_lf; %scale load
mpc_prev.gen(:,2:3) = mpc_prev.gen(:,2:3)*prev_lf; %scale generation
%create previous measurements
pf_results_prev = runpf(mpc_prev);
[measure_prev,idx_prev,sigma_prev,~,~] = simulate_SCADA_meters(pf_results_prev, noise_level);
%get previous SE result (attacker can steal this solution)
baseMVA = mpc_prev.baseMVA;
bus_prev = mpc_prev.bus;
gen_prev = mpc_prev.gen;
branch_prev = mpc_prev.branch;
[success_prev, Vest_prev, ~, ~, ~, ~, ~, ~,~]=...
                                    SE_unit(baseMVA, bus_prev, gen_prev, branch_prev,...
                                            measure_prev, idx_prev, sigma_prev, 100, ...
                                            1);%get operator's estimation at t-1
%% attacker side - prepare inaccuracy models:
% control parameters for imperfect grid model: topology error/inaccurate network parameter
K_topo = 5; % topology inaccuracy
K_para = 2;
Mag_sparse_para = 0.2;
Mag_dense_para = 0.05;
K_x = 0;

%grid model [baseMVA, bus_as, gen_as, branch_as]
baseMVA = mpc.baseMVA;
bus_as = mpc.bus;
gen_as = mpc.gen;
branch_as0 = mpc.branch;
Vest_as0 = Vest_prev;
%topology inaccuracy
shuffled_br_ids_topo = randperm(length(branch_as0));
[branch_as_topo, topo_ids] = get_inaccurate_topology(branch_as0,K_topo,shuffled_br_ids_topo);
%dense network parameter inaccuracy
branch_as_dense = get_inaccurate_para_dense(branch_as0,Mag_dense_para);
%sparse network parameter inaccuracy
shuffled_br_ids_para = randperm(length(branch_as0));
[branch_as_sparse, ntwpara_ids] = get_inaccurate_para_sparse(branch_as0,K_para,Mag_sparse_para,shuffled_br_ids_para);
%combined: topology + network paremeter inaccuracy(sparse+dense)
branch_as_combined = branch_as_dense;
branch_as_combined(topo_ids,11) = 0;
branch_as_combined(ntwpara_ids,3:4) = branch_as_combined(ntwpara_ids,3:4) + Mag_sparse_para;
% state vector inaccuracy
Vest_as_dense = Vest_as0;
Vest_as_dense(:) = 1+0i;
shuffled_state_ids = randperm(length(Vest_as0));
[Vest_as_sparse, x_ids] = get_inaccurate_state_sparse(Vest_as0,K_x,shuffled_state_ids, ref);
 
%% target bad case, what do we want to mislead the operator to? 
% for AC FDIA
tgt_lf = 0.98;
tgtcase_as = mpc;
tgtcase_as.bus(:,3:4) = mpc.bus(:,3:4)*tgt_lf; %reduce load by 10%
pf_astgt = runpf(tgtcase_as);
% for DC FDIA
dA_as = 3*randn(length(bus_as),1); %for FDIA DC
dA_as(11:end)=0;

%% create instances of inaccuracy models (specify hyperpara)
test_sensitivity = 1;
if test_sensitivity == 0
%test different inaccuracy models
MODE_FDIA_list = {'perfect AC'}; %{'perfect AC','DC'};
inaccuracy_list = {'No inaccuracy',...
                    'topology',...
                    'dense network parameter','sparse network parameter',...
                    'dense state','sparse state','combined'};
i_instance = 1;
for p = 1:length(MODE_FDIA_list)
    for q = 1:length(inaccuracy_list)
        instance.MODE_FDIA = MODE_FDIA_list{p}; %'DC','target AC','perfect'
        instance.inaccuracy = inaccuracy_list{q};
        instance.K_topo = K_topo;
        instance.Mag_dense_para = Mag_dense_para;
        instance.K_para = K_para;
        instance.Mag_sparse_para = Mag_sparse_para;
        instance.K_x = K_x;
        Instances{i_instance} = instance;
        i_instance = i_instance + 1;
    end
end

%% test the sensitivity wrt one type of inaccuracy:
elseif test_sensitivity == 1
    tested_inaccuracy = 'topology';
    i_instance = 1;
    if strcmp(tested_inaccuracy, 'topology')==1
        for K_topo = 0:1:8 %0:8
            instance.MODE_FDIA = 'perfect AC'; %'DC','target AC','perfect AC'
            instance.inaccuracy = tested_inaccuracy;
            instance.K_topo = K_topo;
            Instances{i_instance} = instance;
            i_instance = i_instance + 1;
        end
    elseif strcmp(tested_inaccuracy,'dense network parameter')==1
        for Mag_dense_para = 0:0.01:0.8
            instance.MODE_FDIA = 'perfect AC'; %'DC','target AC','perfect AC'
            instance.inaccuracy = tested_inaccuracy;
            instance.Mag_dense_para = Mag_dense_para;
            Instances{i_instance} = instance;
            i_instance = i_instance + 1;
        end
    elseif strcmp(tested_inaccuracy,'sparse network parameter')==1
        for K_para = 0:1:10
            instance.MODE_FDIA = 'perfect AC'; %'DC','target AC','perfect AC'
            instance.inaccuracy = tested_inaccuracy;
            instance.K_para = K_para;
            instance.Mag_sparse_para = Mag_dense_para;
            Instances{i_instance} = instance;
            i_instance = i_instance + 1; 
        end
    elseif strcmp(tested_inaccuracy, 'sparse state')==1
        for K_x = 0:1:2
            instance.MODE_FDIA = 'perfect AC'; %'DC','target AC','perfect AC'
            instance.inaccuracy = tested_inaccuracy;
            instance.K_x = K_x;
            Instances{i_instance} = instance;
            i_instance = i_instance + 1; 
        end        
    end
end
%% according to MODE_FDIA, inaccuracy, fill in  Vest_as, branch_as
for i = 1:length(Instances)
    instance = Instances{i};
    instance.Vest_as = Vest_as0;
    instance.branch_as = branch_as0;
    if strcmp(instance.inaccuracy,'topology')==1
        %instance.branch_as = branch_as_topo;
        [instance.branch_as, topo_ids] = get_inaccurate_topology(branch_as0,instance.K_topo,shuffled_br_ids_topo);
    elseif strcmp(instance.inaccuracy,'dense network parameter')==1
        %instance.branch_as = branch_as_dense;
        instance.branch_as = get_inaccurate_para_dense(branch_as0,instance.Mag_dense_para);
    elseif strcmp(instance.inaccuracy,'sparse network parameter')==1
        %instance.branch_as = branch_as_sparse;
        [instance.branch_as, ~] = get_inaccurate_para_sparse(branch_as0,instance.K_para,instance.Mag_sparse_para,shuffled_br_ids_para);
    elseif strcmp(instance.inaccuracy,'dense state')==1
        instance.Vest_as = Vest_as_dense;
    elseif strcmp(instance.inaccuracy,'sparse state')==1
        %instance.Vest_as = Vest_as_sparse;
        [instance.Vest_as, ~] = get_inaccurate_state_sparse(Vest_as0,instance.K_x,shuffled_state_ids, ref);
    elseif strcmp(instance.inaccuracy,'combined')==1
        instance.branch_as = branch_as_combined;
        instance.Vest_as = Vest_as_sparse;
    end
    Instances{i} = instance;
end
%% for each instance, generate fake data @inaccuracy model, and run operator's SE
for i = 1:length(Instances)
    instance = Instances{i};
    MODE_FDIA = instance.MODE_FDIA;
    branch_as = instance.branch_as;
    Vest_as = instance.Vest_as;
    if strcmp(MODE_FDIA,'target AC')
        %AC FDIA designed by certain target wrong solution Vtarget_as
        Vtarget_as = Vest_as; %target manipulate on estimate x
        Vtarget_abs = abs(Vest_as); %magnitude of wrong solution
        Vtarget_as=Vtarget_abs.*exp(1i*angle(Vest_as)*2); %Xa, the bad state that attackers want to mislead operators toward
        Vtarget_as(ref)=1+0i; %attackers know reference bus  
        [measure_a, idx] = ...
            fdia_ac_gen(Vest_as,Vtarget_as, measure, idx,...
                        baseMVA, bus_as, gen_as, branch_as);
    elseif strcmp(MODE_FDIA, 'perfect AC')||strcmp(MODE_FDIA, 'DC')
        %well crafted fdia ac
        %Vest_as ready
        if strcmp(MODE_FDIA, 'perfect AC')
        [measure_a, idx, Vtarget_as] = ...
                    fdia_perfac_gen(Vest_as,pf_astgt, ...
                                measure,idx,...
                                   baseMVA, bus_as, gen_as, branch_as);

        elseif strcmp(MODE_FDIA, 'DC')
        %DC FDiA:
        %dA_as = randn(length(bus_as),1); %delta angle (radias), create some random disturb of angle        
        %dA_as = angle(Vest_as)-pf_astgt.bus(:,9)./180.*pi;
        %dA_as(ref)=0;       
        [measure_a, idx] = fdia_dc_gen(dA_as,measure,idx, baseMVA, bus_as, gen_as, branch_as);
        Vtarget_as=abs(V_GT).*exp(1i*(angle(V_GT)+dA_as)); %Xa, the bad state that attackers want to mislead operators toward
        end
    end

    %% operator side: SE unit with BDD and BDI (V_SE is V_a)
    baseMVA = mpc.baseMVA;
    bus = mpc.bus;
    gen = mpc.gen;
    branch = mpc.branch;
    init_option = 1;
    alpha = 100;
    fprintf("used sigma*%d in SE\n",alpha)
    [success, V_SE, z_SEest, z_SEuse, if_bad, J, t_J, rW,rN]=...
                                    SE_unit(baseMVA, bus, gen, branch,...
                                            measure_a, idx, sigma,alpha, ...
                                            init_option);
    [success_NA, V_SE_NA, z_SEest_NA, z_SEuse_NA, if_bad_NA, J_NA, t_J_NA, rW_NA,rN_NA]=...
                                    SE_unit(baseMVA, bus, gen, branch,...
                                            measure, idx, sigma, alpha, ...
                                            init_option); %NA: no attack
    %% operator side: SE accuracy 
    % RMSE
    res_x = sqrt(sum(real(V_SE-V_GT).^2+imag(V_SE-V_GT).^2)/length(bus));
    res_xa = sqrt(sum(real(V_SE-Vtarget_as).^2+imag(V_SE-Vtarget_as).^2)/length(bus));
    fprintf('Accuracy RMSE(x) = %1.6f, RMSE(xa) = %1.6f\n',res_x,res_xa)
    fprintf('BDD %d, J %.4f,tJ %.4f\n',if_bad, J, t_J)

    res_x_NA = sqrt(sum(real(V_SE_NA-V_GT).^2+imag(V_SE_NA-V_GT).^2)/length(bus));
    fprintf('No attack accuracy RMSE(x) = %1.6f\n',res_x_NA)
    fprintf('No attack BDD %d, J %.4f,tJ %.4f\n',if_bad_NA, J_NA, t_J_NA)

    %save in instance
    instance.J = J;
    instance.J_NA = J_NA;
    instance.res_x = res_x;
    instance.res_xa = res_xa;
    instance.res_x_NA = res_x_NA;
    instance.t_J = t_J;
    Instances{i} = instance;
end %end instance

%% final print of all simulated scenarios
disp("==============================\n\n")
for i = 1:length(Instances)
   instance = Instances{i};
   fprintf('%10s,%25s, J=%.2f, J(no attack)=%.2f, %.4f, %.4f, %.4f \n',...
       instance.MODE_FDIA,instance.inaccuracy,...
       instance.J, instance.J_NA,...
         instance.res_x, instance.res_xa,... 
            instance.res_x_NA); 
end

%% plot a curve of J if test sensitivity of a specific inaccuracy
scaling_factor = 1;
if test_sensitivity == 1
    for i = 1:length(Instances)
       Instances{i}.J = Instances{i}.J/scaling_factor; %scaling
       Instances{i}.J_NA = Instances{i}.J_NA/scaling_factor;
       Instances{i}.t_J = nan;
       instance = Instances{i};
       if strcmp(instance.inaccuracy,'topology')==1
            x(i) = instance.K_topo; 
       elseif strcmp(instance.inaccuracy,'dense network parameter')==1
            x(i) = instance.Mag_dense_para; 
        elseif strcmp(instance.inaccuracy,'sparse network parameter')==1
            x(i) = instance.K_para;  
        elseif strcmp(instance.inaccuracy,'sparse state')==1
            x(i) = instance.K_x; 
       end
            
       y(i) = instance.J;
    end    
    
    figure('Position',[100 100 300 200])
    plot(x,y,'--s',...
    'LineWidth',2,...
    'MarkerSize',5,...
    'MarkerEdgeColor','b',...
    'MarkerFaceColor',[0.5,0.5,0.5]) , hold on
    title(['RSS w.r.t. ' num2str(tested_inaccuracy), ' inaccuracy'])
    xlabel(instance.inaccuracy)
    ylabel('RSS')
    
else

    for i = 1:length(Instances)
       Instances{i}.J = Instances{i}.J/scaling_factor;
       Instances{i}.J_NA = Instances{i}.J_NA/scaling_factor;
       Instances{i}.t_J = thre;
      instance = Instances{i};
       x(i) = i;
       J(i) = instance.J;
    end    
    figure('Position',[99 100 200 200])
    bar(1,instance.J_NA,'FaceColor',[0.5 .5 .5]), hold on
    bar(x+1,J), hold on
    %title(['RSS'])
    %xlabel('')
    ylabel('RSS')
end

%save('myinstances.mat','Instances')
