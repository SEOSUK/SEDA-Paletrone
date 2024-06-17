%%Series Elastic Actuator  

   %                      c1                    c2
   %    k1       |======|--]--        |======|--]--
   %--/-/-/-/-/--|  j1  |--/-/-/-/-/--|  j2  |
   %             |======|     k2      |======|
   % ______________________________________

% joint :: O 
% EE :: X
% 
% O---------O------------X
% |<------->|<----------->
%     L1          L2


% link value
L1 = 0.20075; %m
L2 = 0.149;   %m
m1 = 0.340;   %kg
m2 = 0.140;   %kg
mEE = 0.07;   %kg
CoM1 = 0.117; %m
CoM2 = 0.045; %m

%SEA value
k1 = 5.9*10^3; %N/m
k2 = 5.9*10^3; %N/m

c1 = 0.6; %ma.um.dea.ro value
c2 = 0.6;%ma.um.dea.ro value

J1 = m1*L1^2/12 + m1*CoM1^2;                     %kgm^2
J2 = m2*L2^2/12 + m2*CoM2^2 + (mEE*(L2-CoM2)^2); %kgm^2

%ref value
Fx = 0;
Fy = 0;
Px = 0.1;
Py = 0.05;

%MCG value
g = 9.81;
N1 = m1*CoM1^2 + J1 + m2*L1^2 + m2*CoM2^2 + J2;
N2 = m2*L1*CoM2;
N3 = m2*CoM2^2 + J2;

%State-Space Matrix
A = [0               1      0      0;
    -(k1+k2)/J1   -c1/J1   k2/J1   0;
     0               0      0      1;
    k2/J2            0   -k2/J2 -c2/J2];

B = [0   0;
    1/J1 0;
     0   0;
     0  1/J1];

C = eye(4);

D = [0 0;
     0 0;
     0 0;
     0 0];

% %MPC
plant = ss(A,B,C,D); %상태공간
mpcmodel = setmpcsignals(plant, 'MV', 2, 'MO', 2); % MPC의 모델밸류(인풋), 모델아웃풋(아웃풋) 갯수
MV = struct(Min= -5, Max= 5);
% MV = horzcat(MV_1, MV_2);
% OV_1 = struct('Min', -0.1, 'Max', 0.1, 'ScaleFactor', 0.2);
% OV_2 = struct('Min', -0.1, 'Max', 0.1, 'ScaleFactor', 0.2);
% Weights_1 = struct('MV_1', 1, 'MVRate', 0.1, 'OV_1', 1); % 가중치 설정쪽. MVRate: 샘플타임, OV: 아웃풋밸류 가중치
% Weights_2 = struct('MV_2', 1, 'MVRate', 0.1, 'OV_2', 1); % 가중치 설정쪽. MVRate: 샘플타임, OV: 아웃풋밸류 가중치
% Weights = horzcat(Weights_1, Weights_2);
% 
 Ts = 0.01; % 샘플링타임
 p = 10; % 예측출력수
 m = 3; % 예측입력수
 mpcobj = mpc(mpcmodel, Ts, p, m, [], MV);

%% Control section

% PID gain
k_p = 1;
k_d = 0.1;

%Transfer function
num = [0 k_d*j k_d*c+j*k_p c*k_p-k_d*k_sp -k_sp*k_p];
den = [J*j J*c+j*C C*c-J*k_sp-j*k_sp -c*k_sp-C*k_sp 0]+[0 k_d*j k_d*c+j*k_p c*k_p-k_d*k_sp -k_sp*k_p];

tf(num,den)

%stabillity
rlocus(num, den)

