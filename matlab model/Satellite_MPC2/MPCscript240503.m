%% create MPC controller object with sample time
mpc_per_exclude = mpc(plant_per_exclude_C, 0.001);
%% specify prediction horizon
mpc_per_exclude.PredictionHorizon = 15;
%% specify control horizon
mpc_per_exclude.ControlHorizon = 3;
%% specify nominal values for inputs and outputs
mpc_per_exclude.Model.Nominal.U = [0;0;0];
mpc_per_exclude.Model.Nominal.Y = [7367700;0.0039866;1.44728984760677;5.18870678654646;4.76384303206273];
%% specify constraints for MV and MV Rate
mpc_per_exclude.MV(1).Min = -0.0037;
mpc_per_exclude.MV(1).Max = 0.0037;
mpc_per_exclude.MV(2).Min = -0.0037;
mpc_per_exclude.MV(2).Max = 0.0037;
mpc_per_exclude.MV(3).Min = -0.0037;
mpc_per_exclude.MV(3).Max = 0.0037;
%% specify overall adjustment factor applied to weights
beta = 0.13534;
%% specify weights
mpc_per_exclude.Weights.MV = [0 0 0]*beta;
mpc_per_exclude.Weights.MVRate = [0.1 0.1 0.1]/beta;
mpc_per_exclude.Weights.OV = [1 1 0.01 0 0]*beta;
mpc_per_exclude.Weights.ECR = 100000;
%% specify simulation options
options = mpcsimopt();
options.InputNoise = mpc_per_exclude_LDSignal;
options.RefLookAhead = 'off';
options.MDLookAhead = 'off';
options.Constraints = 'on';
options.OpenLoop = 'off';
%% run simulation
sim(mpc_per_exclude, 7200001, mpc_per_exclude_RefSignal, mpc_per_exclude_MDSignal, options);
