function [pf_astgt,tgtcase_as] = get_attack_target_case(mpc_as,tgt_lf)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
tgtcase_as = mpc_as;
tgtcase_as.bus(:,3:4) = tgtcase_as.bus(:,3:4)*tgt_lf; %reduce load by 10%
pf_astgt = runpf(tgtcase_as);
end

