sim_type = 'topology';
[x,y] = run_sim(sim_type);

data = cat(1, x, y);

samples = 1000;
for i = 1:samples
    [x, y] = run_sim(sim_type);
    data = cat(1, data, y);
end

writematrix(data,'ACTIVSg_topology.csv') 
% figure('Position',[100 100 300 200])
% plot(x,y,'--s',...
% 'LineWidth',2,...
% 'MarkerSize',5,...
% 'MarkerEdgeColor','b',...
% 'MarkerFaceColor',[0.5,0.5,0.5]) , hold on
% plot(x,tJ,'r','LineWidth',1) 
% title(['RSS w.r.t. ' num2str(tested_inaccuracy), ' inaccuracy'])
% xlabel(instance.inaccuracy)
% ylabel('RSS')
function [x, y] = run_sim(sim_type)
    % Run once to setup data
    FDIA_modelling_main

    % Export data
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
        end
        y(i) = instance.J;
    end
end