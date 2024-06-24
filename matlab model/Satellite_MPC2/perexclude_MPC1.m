%% create MPC controller object with sample time
mpc1 = mpc(plant_C, 0.001);
%% specify prediction horizon
mpc1.PredictionHorizon = 50;
%% specify control horizon
mpc1.ControlHorizon = 10;
%% specify nominal values for inputs and outputs
mpc1.Model.Nominal.U = [0;0;0;0;0;0];
mpc1.Model.Nominal.Y = [7367700;0.0039866;1.44728984760677;5.18870678654646;4.76384303206273];
%% specify constraints for OV
mpc1.OV(1).Min = 6378200;
mpc1.OV(2).Min = 0;
mpc1.OV(2).Max = 1;
mpc1.OV(3).Min = 0;
mpc1.OV(3).Max = 6.28318530717959;
mpc1.OV(4).Min = 0;
mpc1.OV(4).Max = 6.28318530717959;
mpc1.OV(5).Min = 0;
mpc1.OV(5).Max = 6.28318530717959;
%% specify overall adjustment factor applied to weights
beta = 0.13534;
%% specify weights
mpc1.Weights.MV = [0 0 0]*beta;
mpc1.Weights.MVRate = [0.1 0.1 0.1]/beta;
mpc1.Weights.OV = [1 1 1 0 0]*beta;
mpc1.Weights.ECR = 100000;
%% specify simulation options
options = mpcsimopt();
options.UnmeasuredDisturbance = mpc1_UDSignal;
options.RefLookAhead = 'off';
options.MDLookAhead = 'off';
options.Constraints = 'on';
options.OpenLoop = 'off';
%% run simulation
sim(mpc1, 1200001, mpc1_RefSignal, mpc1_MDSignal, options);
