function [sys,x0,str,ts] = DC(t,x,u,flag)


switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes();

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%
  % Unhandled flags %
  %%%%%%%%%%%%%%%%%%%
  case { 2, 4, 9 },
    sys = [];

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end
% end csfunc

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes()

sizes = simsizes;
sizes.NumContStates  = 2;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2;
sizes.NumInputs      = 2;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0  = zeros(2,1);
str = [];
ts  = [0 0];

% end mdlInitializeSizes
%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u)
%% Place system here 
r=2;
d=2;
ke1 = 0.01;
ke2 = 0.01;
jeq1 = 0.01;
jeq2 = 0.012;
beq1 = 0.1;
beq2 = 0.11;
kt1 = 0.3;
kt2 = 0.35;
Ra1 = 1;
Ra2 = 1.1;
%change of variables
ur=x(1); %omega r
ul=x(2); %omega l
Vr=u(1); %right voltage
Vl=u(2); %left voltage

%system states sys is your state derivatives

urtransfer = (jeq1*Ra1)/(beq1*Ra1 + kt1*ke1);
Vrtransfer = kt1/(beq1*Ra1 + kt1*ke1);
ultransfer = (jeq2*Ra2)/(beq2*Ra2 + kt2*ke2);
Vltransfer = kt2/(beq2*Ra2 + kt2*ke2);

sys(1) = (-ur + Vrtransfer*Vr)/urtransfer;
sys(2) = (-ul + Vltransfer*Vl)/ultransfer;

%sys = A*x + B*u;

% end mdlDerivatives
%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)
C=eye(2);
sys = C*x;

% end mdlOutputs