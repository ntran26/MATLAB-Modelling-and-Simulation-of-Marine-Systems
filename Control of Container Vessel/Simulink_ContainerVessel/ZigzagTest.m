function [sys,x0,str,ts] = ZigzagTest(t,x,u,flag)

% ZigzagTest.m, Version 1.1 This is a program which does a zigzag test 
% to generate data to estimate parameters based on the Recursive
% Prediction Error Method (Ljung and W.W Zhou)

format compact

switch flag
   
case 0
   [sys,x0,str,ts] = mdlInitializeSizes;
%case 2
%   [sys,x0,str,ts] = mdlUpdate(t,x,u,lambda,P,xi);
case 3
sys = mdlOutputs(t,x,u);
case { 1, 2, 4, 9}
   sys = [];
otherwise
   error(['Unhandled flag = ',num2str(flag)]);
end
% End of function myfun

function [sys,x0,str,ts] = mdlInitializeSizes

sizes = simsizes;

sizes.NumContStates = 0;
sizes.NumDiscStates = 0;
sizes.NumOutputs = 1;
sizes.NumInputs = -1;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0 = [];
str = [];
ts = [-1 0];

% end of mdlInitializeSizes

function sys = mdlOutputs(t,x,u)

%
% This function generates the rudder order for zigzag test.

% Simulate zig-zag manoeuvers
% u(1) = rudder in radian
% u(3) = yaw in deg

rudder = u(1);

if  u(2) >= u(1)
      rudder = -u(1);
elseif u(3) < 0;
      rudder = -u(1);
         if u(2) <= -u(1)
             rudder = u(1);
         elseif u(3) > 0
            rudder = u(1);
         end
end;

sys = [rudder];
         
% End of function mdlOutputs