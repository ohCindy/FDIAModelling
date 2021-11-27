N_trial = 10;
Y_all = [];
S_all = [];

for i_trial = 1:N_trial
    FDIA_modelling_main;
    %save(sprintf('%s_y_%d.mat',casename(1:end-2),i_trial),'y')
    Y_all = [Y_all;y_J]; 
    S_all = [S_all;y_suc];
    %save(sprintf('%s_Yall_%d.mat',casename(1:end-2),i_trial),'Y_all')
    %save(sprintf('%s_Sall_%d.mat',casename(1:end-2),i_trial),'S_all')
end

% save(sprintf('%s_Yall_%d.mat',casename(1:end-2),i_trial),'Y_all')
% save(sprintf('%s_Sall_%d.mat',casename(1:end-2),i_trial),'S_all')

% for i_trial = 1:N_trial
%     load(sprintf('%s_y_%d.mat',casename(1:end-2),i_trial),'y')
%     Y_all = [Y_all;y];
% end

%figure,plot(mean(Y_all)),hold on
%plot(Y_all','bo')

res = cat(1, x, Y_all);
csvwrite('result_J.csv', res)
res = cat(1, x, S_all);
csvwrite('result_S.csv', res)

% figure, boxplot(S_all)
% xticks(1:length(x))
% xticklabels(x)

