function [sys,x0,str,ts] = diffdrive(t,x,u,flag)


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
sizes.NumContStates  = 5;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 5;
sizes.NumInputs      = 2;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0  = zeros(5,1);
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
%change of variables
x1=x(1); %x
y=x(2); %y
phi=x(3); %phi
ur=x(4); %omega r
ul=x(5); %omega l
Vr=u(1); %right voltage
Vl=u(2); %left voltage

%system states sys is your state derivatives

urtransfer = (jeq1*Ra1)/(beq1*Ra1 + kt1*ke1);
Vrtransfer = kt1/(beq1*Ra1 + kt1*ke1);
ultransfer = (jeq2*Ra2)/(beq2*Ra2 + kt2*ke2);
Vltransfer = kt2/(beq2*Ra2 + kt2*ke2);

sys(1) = (r/2)*(ur + ul)*cos(phi);
sys(2) = (r/2)*(ur + ul)*sin(phi);
sys(3) = (r/(2*d))*(ur - ul);
sys(4) = (-ur + Vrtransfer*Vr)/urtransfer;
sys(5) = (-ul + Vltransfer*Vl)/ultransfer;

%sys = A*x + B*u;

% end mdlDerivatives
%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)
C=eye(5);
sys = C*x;

% end mdlOutputs
