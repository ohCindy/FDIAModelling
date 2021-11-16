function [V, baseMVA, bus, gen, branch, success,...
    et, z, z_est, error_sqrsum,...
    varargout] = ...
    run_se(baseMVA, bus, gen, branch, measure, idx, sigma, type_initialguess, z_truth, V_GT)
%RUN_SE  Run state estimation.
%   [INPUT PARAMETERS]
%   measure: measurements
%   idx: measurement indices
%   sigma: measurement variances
%   [OUTPUT PARAMETERS]
%   z: Measurement Vector. In the order of PF, PT, PG, Va, QF, QT, QG, Vm (if
%   applicable), so it has ordered differently from original measurements
%   z_est: Estimated Vector. In the order of PF, PT, PG, Va, QF, QT, QG, Vm
%   (if applicable)
%   error_sqrsum: Weighted sum of error squares
%   created by Rui Bo on 2007/11/12

%   MATPOWER
%   Copyright (c) 1996-2016, Power Systems Engineering Research Center (PSERC)
%   by Rui Bo
%   and Ray Zimmerman, PSERC Cornell
%
%   This file is part of MATPOWER/mx-se.
%   Covered by the 3-clause BSD License (see LICENSE file for details).
%   See https://github.com/MATPOWER/mx-se/ for more info.

%% define named indices into data matrices
[GEN_BUS, PG, QG, QMAX, QMIN, VG, MBASE, GEN_STATUS, PMAX, PMIN, ...
    MU_PMAX, MU_PMIN, MU_QMAX, MU_QMIN, PC1, PC2, QC1MIN, QC1MAX, ...
    QC2MIN, QC2MAX, RAMP_AGC, RAMP_10, RAMP_30, RAMP_Q, APF] = idx_gen;
[PQ, PV, REF, NONE, BUS_I, BUS_TYPE, PD, QD, GS, BS, BUS_AREA, VM, ...
    VA, BASE_KV, ZONE, VMAX, VMIN, LAM_P, LAM_Q, MU_VMAX, MU_VMIN] = idx_bus;
%% read data & convert to internal bus numbering
%[baseMVA, bus, gen, branch] = loadcase(casename);
[i2e, bus, gen, branch] = ext2int(bus, gen, branch);

%% get bus index lists of each type of bus
[ref, pv, pq] = bustypes(bus, gen);

%% build admittance matrices
[Ybus, Yf, Yt] = makeYbus(baseMVA, bus, branch);
Ybus = full(Ybus);
Yf = full(Yf);
Yt = full(Yt);

%% prepare initial guess
% if nargin < 6
%     
% else
%     V0 = getV0(bus, gen, type_initialguess, V0);
% end
V0 = getV0(bus, gen, type_initialguess);
%% run state estimation
t0 = tic;
[V, success, iterNum, z, z_est, error_sqrsum, H] = ...
    doSE(baseMVA, bus, gen, branch,...
    Ybus, Yf, Yt, V0, ref, pv, pq, measure, idx, sigma);

% update Pg and Qg using estimated values
gbus = gen(:, GEN_BUS);
Sgbus = V(gbus) .* conj(Ybus(gbus, :) * V);
Sgen = Sgbus * baseMVA + (bus(gbus, PD) + 1j*bus(gbus, QD));    %% inj S + local Sd
gen(:, PG) = real(Sgen);
gen(:, QG) = imag(Sgen);
%% update data matrices with solution, ie, V
% [bus, gen, branch] = updatepfsoln(baseMVA, bus, gen, branch, Ybus, V, ref, pv, pq);
[bus, gen, branch] = pfsoln(baseMVA, bus, gen, branch, Ybus, Yf, Yt, V, ref, pv, pq);
et = toc(t0);

%%-----  output results  -----
%% convert back to original bus numbering & print results
[bus, gen, branch] = int2ext(i2e, bus, gen, branch);
%% output power flow solution
%outputpfsoln(baseMVA, bus, gen, branch, success, et, 1, iterNum);
%% output state estimation solution
%outputsesoln(idx, sigma, z, z_est, error_sqrsum);

if max(nargout,1) > 10
    varargout{1} = H;
end
