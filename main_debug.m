N_trial = 3;
Y_all = [];

for i_trial = 1:N_trial
    FDIA_modelling_main;
    %save(sprintf('%s_y_%d.mat',casename(1:end-2),i_trial),'y')
    Y_all = [Y_all;y];
    %save(sprintf('%s_Yall_%d.mat',casename(1:end-2),i_trial),'Y_all')
end

% for i_trial = 1:N_trial
%     load(sprintf('%s_y_%d.mat',casename(1:end-2),i_trial),'y')
%     Y_all = [Y_all;y];
% end

%figure,plot(mean(Y_all)),hold on
%plot(Y_all','bo')

% figure, boxplot(Y_all)
% 
% xticks(1:length(y))
% xticklabels(x)
res = cat(1, x, Y_all);
csvwrite('result.csv', res)


