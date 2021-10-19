%% ADVANCED ELECTRICAL DRIVES MINI-PROJECT
%--------------------------------------------------------------------------
% Operating Trajectories for the Control of Permanent Magnet Synchronous
% Machines

%%
%--------------------------------------------------------------------------
% Consideration to use the Look Up Table Script (LUT):

% If this script is being called to compute the Look Up Table with different
% machine parameters,kindly comment the parameter initialization under "INITIALIZATION of Machine Parameters block" 
% and make sure that the same naming convention is used for the respective parameters.

% The variable 'k' in the code denotes the maximum operating point for a given Torque at
% MTPA line (point A), varible 'l' denotes the maximum operating point at
% reference torque line (point B), variable 'r' is the maximum operating point
% at MA circle (point C).

% Operating point i_sd and i_sq for a given torque and given speed are
% printed in the command window as a part of this script. If this script is
% run stand alone, the returned points will be as per the parameter values
% initialized in the script.

% Currents(isd_LUT, isq_LUT), Speed in rad/s(w_s), Operating Electrical Torque(T_e_op) 
% Operating mechanlical Torue(T_m) and Speed in rpm(n) constitues the
% necesarry look up table. In case, a plotting script is designed to plot
% the oeprating regions, the above mentioned vectors form the LUT for the
% complete operating region.

%--------------------------------------------------------------------------
% INITIALIZATION of Machine Parameters
%--------------------------------------------------------------------------

% clear all; close all; clc;
% 
% res = 7500; %resolution
% 
% Psi_f = 90e-3;      % Field flux linkage
% L_sd = 200e-6;       % Stator inductance in d-axis
% L_sq = 500e-6;       % Stator inductance in q-axis
% i_smax = 500;     % Maximum stator current (amplitude)
% U_dc = 350;       % Dc-link voltage
% u_smax = U_dc/sqrt(3);     % Maximum stator voltage
% p = 4;          % Pole pairs
% i_sd = 0;       % Stator current in d-axis
% i_sdn = 0;     % Stator current in d-axis (normalized on ismax)
% i_sq = 0;      % Stator current in q-axis
% i_sqn = 0;      % Stator current in q-axis (normalized on ismax)
% i_sc = Psi_f / L_sd;       % Short circuit current
% kappa = i_sc / i_smax;      % Short circuit current normalized on ismax
% T_e = 70;        % Electrical torque
% T_m = 0;        % Mechanical torque
% omega_s = 20000;    % Electrical angular velocity
% n = 0;          % Speed in rpm
% chi = (L_sq - L_sd)/(2 * L_sd);  %Saliency
%--------------------------------------------------------------------------

n_max = 50000; %maximum speed of the speed range in rpm

isd_LUT = zeros(res,1);       % Stator current in d-axis
isdn_LUT = zeros(res,1);     % Stator current in d-axis (normalized on ismax)
isq_LUT = zeros(res,1);      % Stator current in q-axis
isqn_LUT = zeros(res,1);      % Stator current in q-axis (normalized on ismax)


%% 1.1 b)
%--------------------------------------------------------------------------
% Script that generaT_es the reference currents isd and isq for a specific
% point (T_e, ws) to operaT_e a generic Permanent Magnet Synchronous Machine
% safe and efficiently in base speed operation, basic field weakening and
% field weakening. The reference currents are calculaT_ed offline and stored
% in lookup tables (LUT)
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% inT_ernal variable initiazlization
%--------------------------------------------------------------------------

isq_circle=zeros(res,1);    %initialization of isq for the MA Circle with zeros
isd_circle=linspace(0,-i_smax,res)';    %initialization of isd for the MA Circle with evenly distribuT_ed current values from 0-i_smax
isqn_circle=zeros(res,1);   %initialization of isqn for the MA Circle
isdn_circle=zeros(res,1);   %initialization of isdn for the MA Circle

isq_mf=zeros(res,1);    %initialization of isq for the MF curves
isd_mf=zeros(res,1);    %initialization of isd for the MF curves

isq_mtpa=zeros(res,1);  %initialization of isq for the MTPA curves
isd_mtpa=zeros(res,1);  %initialization of isd for the MTPA curves
isqn_mtpa=zeros(res,1); %initialization of isqn for the MTPA curves
isdn_mtpa=zeros(res,1); %initialization of isdn for the MTPA curves
is_mtpa = zeros(res,1); %initialization of is for the MTPA curves

T_e_mtpa=zeros(res,1);   %Torque values on MTPA Line
isd_Tmax = 0;   %initialization of isd for maximum Torque
isq_Tmax = 0;   %initialization of isq for maximum Torque
T_e_max=0;   %initialization of maximum Torque


io=zeros(res,1);    %initialization of io for MTPF calculations

isq_mtpf=zeros(res,1);  %initialization of isq for the MTPF curves
isd_mtpf=zeros(res,1);  %initialization of isd for the MTPF curves
isqn_mtpf=zeros(res,1); %initialization of isqn for the MTPF curves
isdn_mtpf=zeros(res,1); %initialization of isdn for the MTPF curves
is_mtpf = zeros(res,1); %initialization of is for the MTPF curves

u_s=linspace(0,u_smax,res)';    %initialization of voltage buildup from 0-u_smax


%--------------------------------------------------------------------------
% MA Cicrcle, MF , MTPA, MTPF curves
%--------------------------------------------------------------------------

for j=1:res
    
    % MA Cicrcle
    %--------------------------------------------------------------------------
    isdn_circle(j)=isd_circle(j)/i_smax;
    isq_circle(j)=sqrt(i_smax^2-(isd_circle(j))^2); %MA circle function solved for isq
    isqn_circle(j)=isq_circle(j)/i_smax;
    
    %--------------------------------------------------------------------------
    
    % MTPA Curve
    %--------------------------------------------------------------------------
    isd_mtpa(j)=isd_circle(j);
    isdn_mtpa(j)=isd_mtpa(j)/i_smax;
    is_mtpa(j)= sqrt(2*(((kappa/(8*chi))-isdn_mtpa(j))^2 - (kappa/(8*chi)^2)))*i_smax;
    
    isq_mtpa(j) = sqrt(is_mtpa(j)^2 - isd_mtpa(j)^2); %MTPA curve function solved for isq
    isqn_mtpa(j) = isq_mtpa(j)/i_smax;
    
    if(is_mtpa(j) <= i_smax)
        isd_Tmax = isd_mtpa(j);
        isq_Tmax = isq_mtpa(j);
        s=j;
    end
    
    T_e_mtpa(j) = 1.5*isq_mtpa(j,1)*(1 - 2*(chi*isd_mtpa(j,1)/i_sc))*Psi_f;
    
    
    T_e_max = 1.5*isq_Tmax*(1 - 2*(chi*isd_Tmax/i_sc))*Psi_f;
    ws_base = (u_smax/(L_sd*i_smax))/(sqrt(((isd_Tmax/i_smax)+kappa)^2 + ((isq_Tmax/i_smax)*(2*chi + 1))^2));
    %--------------------------------------------------------------------------
end

for j=1:res
    
    % MF Curve
    %--------------------------------------------------------------------------
    isd_mf(j) = isd_circle(j);
    isq_mf(j) = sqrt((u_smax/ws_base)^2 - (Psi_f + L_sd*isd_mf(j))^2)/L_sq;
    
    % MTPF Curve
    %--------------------------------------------------------------------------
    c1 = (1 + 2*chi);
    c2 = c1*kappa/(8*chi);
    isd_mtpf(j)=isd_circle(j);
    isdn_mtpf(j)=isd_mtpf(j)/i_smax;
    io(j) = sqrt(2*((-kappa + c2 - isdn_mtpf(j))^2 - c2^2));
    isqn_mtpf(j) = sqrt(io(j)^2 -(kappa+isdn_mtpf(j))^2)/c1;
    isq_mtpf(j) = isqn_mtpf(j)*i_smax;   %MTPF curve function solved for isq
    
    is_mtpf(j) = sqrt(isd_mtpf(j)^2 + isq_mtpf(j)^2);
    
    if(is_mtpf(j) <= i_smax)
        r=j;
    end
    
    if(abs(isd_mtpf(j)) <= i_sc)
        t=j;
    end
    %--------------------------------------------------------------------------
    
end
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Speed values
%--------------------------------------------------------------------------

for j=1:res
    
    % speed on the MTPA curve
    %--------------------------------------------------------------------------
    ws_mtpa(j,1)=(u_smax/(L_sd*i_smax))/sqrt((isdn_mtpa(j,1)+kappa)^2 + (isqn_mtpa(j,1)*(2*chi + 1))^2);
    %--------------------------------------------------------------------------
    
    % speed on the MA Circle
    %--------------------------------------------------------------------------
    ws_ma(j,1)=(u_smax/(L_sd*i_smax))/sqrt((isdn_circle(j,1)+kappa)^2 + (isqn_circle(j,1)*(2*chi + 1))^2);
    %--------------------------------------------------------------------------
    
    % speed on the MTPF curve
    %-------------------------------------------------------------------------
    ws_mtpf(j,1)= (u_s(j)/(L_sd*i_smax))/(sqrt((isdn_mtpf(j,1)+kappa)^2 + (isqn_mtpf(j,1)*(2*chi + 1))^2));
    %--------------------------------------------------------------------------
end
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Torque Lines
%--------------------------------------------------------------------------
isd_maxtorque = isd_circle;
isq_maxtorque = zeros(res,1);

isd_torque = isd_circle;
isq_torque = zeros(res,1);
is_torque = zeros(res,1);

con = 2*(T_e_max)/(3*Psi_f);
con1 = 2*(T_e_max/2)/(3*Psi_f);

for j=1:res
    isq_maxtorque(j,1) = con/(1 - (2*chi*isd_maxtorque(j,1)/i_sc));
    isq_torque(j,1) = con1/(1 - (2*chi*isd_torque(j,1)/i_sc));
    is_torque(j) = sqrt(isd_torque(j)^2 + isq_torque(j)^2);
end


%--------------------------------------------------------------------------
% Operating torque line
%--------------------------------------------------------------------------
isd_tq_op = isd_circle; %initialization of isd for operating Torque
isq_tq_op = zeros(res,1);   %initialization of isq for operating Torque
is_tq_op = zeros(res,1);    %initialization of is for operating Torque

con2 = 2*(T_e)/(3*Psi_f);

for j=1:res
    isq_tq_op(j,1) = con2/(1 - (2*chi*isd_tq_op(j,1)/i_sc));
    is_tq_op(j) = sqrt(isd_tq_op(j)^2 + isq_tq_op(j)^2);
end

%--------------------------------------------------------------------------
%--------------------------------------------------------------------------



%--------------------------------------------------------------------------
% 0->A: Base speed operation (MTPA)
%--------------------------------------------------------------------------
% Checing where the MTPA line inT_ersects the Desired Torque Line
%--------------------------------------------------------------------------
for j=1:res
    if(isq_tq_op(j)>=isq_mtpa(j))
        k = j;
    end
end
%--------------------------------------------------------------------------
% Operating current values & Speed matrix
%--------------------------------------------------------------------------
for j=1:k
    isd_LUT(j) = isd_mtpa(j);
    isq_LUT(j) = isq_mtpa(j);
    Psi_s1 = sqrt((Psi_f+ (L_sd*isd_LUT(j)))^2 + (L_sq*isq_LUT(j))^2);
    w_s(j) = u_s(j)/Psi_s1;
end

%--------------------------------------------------------------------------



%--------------------------------------------------------------------------
% A->B: Field Weakening Operation (TL)
%--------------------------------------------------------------------------
% Checing where the operating current on the Torque line is within range
%--------------------------------------------------------------------------
for j=k:res
    if(is_tq_op(j)<=i_smax)
        l = j;
    end
end
%--------------------------------------------------------------------------
% Operating current values & Speed matrix
%--------------------------------------------------------------------------
for j=k:l
    isd_LUT(j) = isd_tq_op(j);
    isq_LUT(j) = isq_tq_op(j);
    Psi_s2 = sqrt((Psi_f+ (L_sd*isd_LUT(j)))^2 + (L_sq*isq_LUT(j))^2);
    w_s(j) = u_smax/Psi_s2;
end

%--------------------------------------------------------------------------



%--------------------------------------------------------------------------
% B->C: Field Weakening Operation (MA)
%--------------------------------------------------------------------------
% Operating current values & Speed matrix
%--------------------------------------------------------------------------
for j=l:r
    isd_LUT(j) = isd_circle(j);
    isq_LUT(j) = isq_circle(j);
    Psi_s3 = sqrt((Psi_f+ (L_sd*isd_LUT(j)))^2 + (L_sq*isq_LUT(j))^2);
    w_s(j) = u_smax/Psi_s3;
end

%--------------------------------------------------------------------------



%--------------------------------------------------------------------------
% C->D: Field Weakening Operation (MTPF)
%--------------------------------------------------------------------------
% Operating current values & Speed matrix
%--------------------------------------------------------------------------
for j=1:(r-t-1)
    isd_LUT(r+j) = isd_mtpf(r-j);
    isq_LUT(r+j) = isq_mtpf(r-j);
    Psi_s4 = sqrt((Psi_f+ (L_sd*isd_LUT(r+j)))^2 + (L_sq*isq_LUT(r+j))^2);
    w_s(r+j) = u_smax/Psi_s4;
end
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Current vs Normalized speed curve
%--------------------------------------------------------------------------

isdn_LUT = isd_LUT/i_smax;    %normalizing operating isd
isqn_LUT = isq_LUT/i_smax;    %normalizing operating isd
wn = w_s/ws_base;     %normalizing operating speed

n = omega_s*60/(2*pi*p);       %converting omega into rpm
n_table = w_s*60/(2*pi*p);       %converting omega into rpm

size_wn = size(wn,2);
T_m = zeros(1,size_wn);
T_e_op = zeros(1,size_wn);

for i=1:size_wn
        T_e_op(i) = 3/2*(isq_LUT(i)*Psi_f+(L_sd-L_sq)*isq_LUT(i)*isd_LUT(i));
end
T_m = p*T_e_op;


%% 1.2 a.)

% Checking the maximum speed in the operating speed matrix derived above to
% locat the set of operating currents
for j =1:size(w_s,2)
    if(omega_s>=w_s(j))
        op = j;     %operating point at the resolution
    end
end

fprintf('The operating point current for the reference Torque %dN-m and the reference speed %drad/s is \n',T_e,omega_s);

i_sd=isd_LUT(op)
i_sdn=i_sd/i_smax;
i_sq=isq_LUT(op)
i_sqn=i_sq/i_smax;

isd_op = isd_LUT(1:op);   %values of isd for the T_e_op = 70N-m and omega = 0-50000rpm
isq_op = isq_LUT(1:op);   %values of isq for the T_e_op = 70N-m and omega = 0-50000rpm


