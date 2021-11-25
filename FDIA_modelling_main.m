%close all
%clear
addpath(genpath('matpower7.0/')) %add all subfolders
casename = 'case30.m';
%% global control parameters
test_sensitivity = 1;
tested_inaccuracy = 'meter access';
attack_use_outdated = 1;
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
        bad_data_ids = [3];
        measure.Pinj(bad_data_ids) = sign(measure.Pinj(bad_data_ids))*(1+abs(measure.Pinj(bad_data_ids)));
    end
end

N_meter = length(measure.Pinj)+length(measure.Qinj)+length(measure.PF)+length(measure.QF)+...
            length(measure.PT)+length(measure.QT)+ length(measure.Vm);
shuffled_meter_ids = randperm(N_meter);
%% operator's previous SE settings (attackers can steal previous state estimates at t-1)
%control parameters
K_diff_topo = 1; %create the difference in topology between t-1 and t
K_diff_para = 3; %create number of outdated line parameter t-1 and t
Mag_prev_para = 0.1;
prev_lf = rand(1)*0.05+0.95; %randomly sample a previous lf between 100%+/-2.5%
%% find a feasible previous system and simulate powerflow + SE
success = 0;
while ~success
    %randomness
    shuffled_br_ids_topo = randperm(length(mpc.branch));
    shuffled_br_ids_para = randperm(length(mpc.branch));
    %get estimate
    [success,Vest_prev,mpc_prev] = get_manipulated_case(mpc,...
        K_diff_topo,K_diff_para,...
        Mag_prev_para,0,prev_lf,...
         shuffled_br_ids_topo,shuffled_br_ids_para,noise_level,1);
end
%% attacker side - prepare inaccuracy models:
% control parameters for imperfect grid model: topology error/inaccurate network parameter
baseMVA = mpc.baseMVA;
if attack_use_outdated
    K_topo = K_diff_topo; % topology inaccuracy
    K_para = K_diff_para;
    Mag_sparse_para = Mag_prev_para;
    Mag_dense_para = 0.05;
else
    K_topo = 5; % topology inaccuracy
    K_para = 2;
    Mag_sparse_para = 0.2;
    Mag_dense_para = 0.05;
    %recreate randomness
    shuffled_br_ids_topo = randperm(length(mpc.branch));
    shuffled_br_ids_para = randperm(length(mpc.branch));
end
%% attacker side: attack target, what do we want to mislead the operator to? 
% for AC FDIA, mislead the operator into thinking load changes by tgt_lf
tgt_lf = 1.2;
% for DC FDIA, mislead the operator into thinking there's angle instability
dA_as = 3*randn(length(mpc.bus),1); %for FDIA DC
dA_as(11:end)=0;

%% create instances of inaccuracy models (specify hyperpara)
if test_sensitivity == 0
%test different inaccuracy models
MODE_FDIA_list = {'perfect AC'}; %only perfect AC is available in this code
inaccuracy_list = {'No inaccuracy',...
                    'topology',...
                    'dense network parameter','sparse network parameter','combined'};
i_instance = 1;
for p = 1:length(MODE_FDIA_list)
    for q = 1:length(inaccuracy_list)
        instance.MODE_FDIA = MODE_FDIA_list{p}; %'DC','target AC','perfect'
        instance.inaccuracy = inaccuracy_list{q};
        instance.K_topo = 0;
        instance.K_para = 0;
        instance.Mag_dense_para = 0;
        instance.Mag_sparse_para = 0;
        instance.W_access = 1;
        if strcmp(instance.inaccuracy, 'topology')==1
            instance.K_topo = K_topo;
        elseif strcmp(instance.inaccuracy, 'sparse network parameter')==1
            instance.K_topo = K_para;
            instance.Mag_sparse_para = Mag_sparse_para;
        elseif strcmp(instance.inaccuracy, 'dense network parameter')==1
            instance.Mag_dense_para = Mag_dense_para;
        elseif strcmp(instance.inaccuracy, 'combined')==1
            instance.K_topo = K_topo;
            instance.K_para = K_topo;
            instance.Mag_dense_para = Mag_dense_para;
            instance.Mag_sparse_para = Mag_sparse_para;
        end
        Instances{i_instance} = instance;
        i_instance = i_instance + 1;
    end
end

%% test the sensitivity wrt one type of inaccuracy:
elseif test_sensitivity == 1
    i_instance = 1;
    %% template of instance
        instance.MODE_FDIA = 'perfect AC'; %'DC','target AC','perfect AC'
        instance.inaccuracy = tested_inaccuracy;
        instance.K_topo = 0;
        instance.K_para = 0;
        instance.Mag_dense_para = 0;
        instance.Mag_sparse_para = 0;
        instance.W_access = 1; 
    %%
    if strcmp(tested_inaccuracy, 'topology')==1
        for K_topo = 0:1:8 %0:8
            instance.K_topo = K_topo;
            Instances{i_instance} = instance;
            i_instance = i_instance + 1;
        end
    elseif strcmp(tested_inaccuracy,'dense network parameter')==1
        for Mag_dense_para = 0:0.01:0.1
            instance.Mag_dense_para = Mag_dense_para;
            Instances{i_instance} = instance;
            i_instance = i_instance + 1;
        end
    elseif strcmp(tested_inaccuracy,'sparse network parameter')==1
        for K_para = 0:1:10
            instance.K_para = K_para;
            instance.Mag_sparse_para = Mag_sparse_para;
            Instances{i_instance} = instance;
            i_instance = i_instance + 1; 
        end   
    elseif strcmp(tested_inaccuracy,'meter access')==1 
        for W_access = 1:-0.1:0
            instance.W_access = W_access;
            Instances{i_instance} = instance;
            i_instance = i_instance + 1; 
        end
    end
end
%% according to MODE_FDIA, inaccuracy, fill in  Vest_as, branch_as
for i = 1:length(Instances)
    instance = Instances{i};   
    if strcmp(instance.inaccuracy, 'No inaccuracy')==1 || strcmp(instance.inaccuracy, 'meter access')==1
        [success_NA, instance.Vest_as, ~, ~, ~, ~, ~, ~,~]=...
                                    SE_unit(baseMVA, mpc.bus, mpc.gen, mpc.branch,...
                                            measure, idx, sigma, 100, ...
                                            1); %NA: no attack
        mpc_as = mpc;        
    else
        if attack_use_outdated==1
             [success,instance.Vest_as,mpc_as] = get_manipulated_case(mpc,...
                instance.K_topo,instance.K_para,...
                  instance.Mag_sparse_para,instance.Mag_dense_para,1,...
                     shuffled_br_ids_topo,shuffled_br_ids_para,noise_level,1);
             
        else
            instance.Vest_as = Vest_prev;
            [~,~,mpc_as] = get_manipulated_case(mpc,...
                instance.K_topo,instance.K_para,...
                  instance.Mag_sparse_para,instance.Mag_dense_para,1,...
                     shuffled_br_ids_topo,shuffled_br_ids_para,noise_level,0);
        end
    end
    instance.mpc_as = mpc_as;
    Instances{i} = instance;
end
%% for each instance, according to its inaccuracy model, run operator's SE
for i = 1:length(Instances)
    instance = Instances{i};
    MODE_FDIA = instance.MODE_FDIA;
    Vest_as = instance.Vest_as;
    if strcmp(MODE_FDIA, 'perfect AC')||strcmp(MODE_FDIA, 'DC')
        %well crafted fdia ac
        %Vest_as ready
        if strcmp(MODE_FDIA, 'perfect AC')
            %% generate target according to the attacker's model
            [pf_astgt,tgtcase_as] = get_attack_target_case(instance.mpc_as,tgt_lf);
            %% create fake measurements
            [measure_a, idx, Vtarget_as] = ...
                    fdia_perfac_gen(Vest_as,pf_astgt, ...
                                measure,idx,...
                                   baseMVA, ...
                                   instance.mpc_as.bus, instance.mpc_as.gen, instance.mpc_as.branch);
            if instance.W_access<1
                N_na = ceil((1-instance.W_access)*N_meter);
                if instance.W_access==0
                    measure_a = measure;
                else
                     measure_a = limit_write_access(measure_a,measure,shuffled_meter_ids,N_na);
                end
            end
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
        if i==1
        [success_NA, V_SE_NA, z_SEest_NA, z_SEuse_NA, if_bad_NA, J_NA, t_J_NA, rW_NA,rN_NA,gen_est_NA]=...
                                    SE_unit(baseMVA, bus, gen, branch,...
                                            measure, idx, sigma, alpha, ...
                                            init_option); %NA: no attack
        end
    [success, V_SE, z_SEest, z_SEuse, if_bad, J, t_J, rW,rN, gen_est]=...
                                    SE_unit(baseMVA, bus, gen, branch,...
                                            measure_a, idx, sigma,alpha, ...
                                            init_option);
    % matpower updates the estimated power to generation, instead of load
    %total_load_error = sum(gen_est(:,2)) - sum(gen(:,2));
    total_load_diff = (sum(gen_est(:,2)) - sum(gen_est_NA(:,2)))/sum(gen(:,2));
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
    instance.total_Pload_diff = total_load_diff; % esimtate_attack - estimate_no_attack/estimate_no_attack
    Instances{i} = instance;
end %end instance

%% final print of all simulated scenarios
disp("==============================\n\n")
for i = 1:length(Instances)
   instance = Instances{i};
   fprintf('%10s,%25s, J=%.2f, J(NA)=%.2f, err %.3f, %.3f, %.3f, Pdiff%.2f \n',...
       instance.MODE_FDIA,instance.inaccuracy,...
       instance.J, instance.J_NA,...
         instance.res_x, instance.res_xa,... 
            instance.res_x_NA, instance.total_Pload_diff); 
end
%% prepare for data save
scaling_factor = 1;
for i = 1:length(Instances)
   Instances{i}.J = Instances{i}.J/scaling_factor; %scaling
   Instances{i}.J_NA = Instances{i}.J_NA/scaling_factor;
   Instances{i}.t_J = nan;
   Instances{i}.mpc_as = [];
   Instances{i}.Vest_as = [];
end
%% plot a curve of J if test sensitivity of a specific inaccuracy, save data
plot_on = 0;
    
    if test_sensitivity == 1  
        for i = 1:length(Instances)
           instance = Instances{i};
           if strcmp(instance.inaccuracy,'topology')==1
                x(i) = instance.K_topo; 
           elseif strcmp(instance.inaccuracy,'dense network parameter')==1
                x(i) = instance.Mag_dense_para; 
            elseif strcmp(instance.inaccuracy,'sparse network parameter')==1
                x(i) = instance.K_para;  
            elseif strcmp(instance.inaccuracy,'sparse state')==1
                x(i) = instance.K_x; 
           elseif strcmp(instance.inaccuracy,'meter access')==1
               x(i) = instance.W_access; 
           end           
           y_J(i) = instance.J;
           y_suc(i) = instance.total_Pload_diff;
        end  
        if plot_on
            figure('Position',[100 100 300 200])
            plot(y_J,'--s',...
            'LineWidth',2,...
            'MarkerSize',5,...
            'MarkerEdgeColor','b',...
            'MarkerFaceColor',[0.5,0.5,0.5]) , hold on
            xticks(1:length(y))
            xticklabels(x)
            title(['RSS w.r.t. ' num2str(tested_inaccuracy), ' inaccuracy'])
            xlabel(instance.inaccuracy)
            ylabel('RSS')
        end

    else   
        if plot_on
            for i = 1:length(Instances)
               J(i) = Instances{i}.J;
            end   
            figure('Position',[99 100 200 200])
            bar(1,instance.J_NA,'FaceColor',[0.5 .5 .5]), hold on
            bar((1:length(Instances))+1,J), hold on
            %title(['RSS'])
            %xlabel('')
            ylabel('RSS')
        end
    end

%% save data result
%save(sprintf('%s_instance.mat',casename(1:end-2)),'Instances')
