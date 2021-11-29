addpath(genpath('matpower7.0/')) %add all subfolders

%%
casename = 'case30.m';
N_trial = 200;
lf_min = 0.95;
lf_max = 1.05;

Jbase_list = [];
%%
for i_trial = 1:N_trial
    %% adjust load and gen
    mpc = loadcase(casename);
    lf = rand(1)*(lf_max-lf_min) + 0.5*(lf_max+lf_min);
    mpc.bus(:,3:4) = mpc.bus(:,3:4)*lf; %scale load by lf
    mpc.gen(:,2) = mpc.gen(:,2)*lf; %scale gen by lf
    %% create measurements 
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

    %% 
    baseMVA = mpc.baseMVA;
    bus = mpc.bus;
    gen = mpc.gen;
    branch = mpc.branch;
    init_option = 1;
    alpha = 100;
    [success_NA, V_SE_NA, z_SEest_NA, z_SEuse_NA, if_bad_NA, J_NA, t_J_NA, rW_NA,rN_NA,gen_est_NA]=...
                                        SE_unit(baseMVA, bus, gen, branch,...
                                                measure, idx, sigma, alpha, ...
                                                init_option); %NA: no attack
    Jbase_list(i_trial) = J_NA;                                       
end

figure,boxplot(Jbase_list);
figure,hist(Jbase_list);