%% Final Project ECE414
%Ahmed Almaghasilah 
%Dariya Evans
%First Motor
%% Specifications of the Design
%The ink cartidge starts 2cm before the edge of the paper.  It must travel
%this 2 cm in no more than 160 ms.  Once it reaches the edge of the paper,
%it must travel at 66cm/s with +/-0.01% velocity error tolerance.  After
%traveling 22 cm, the ink cartidge has 2 cm to slow down and stop in no
%more than 160 ms before returning across the page in the opposite
%direction.  The ink cartidge cannot go backward from its initial position
%of zero, nor past the end point 2 cm beyond the far edge of the paper. It
%also cannot strike either of these limits at velocity greater than
%0.25cm/s.  Your analysis need only consider one forward pass across the
%paper since the return trip will have the same response.
%% Plant
%The parts we have to build the system are:
Mi_e = 0.05;     %mass of ink cartidge in kg (empty).
Mi_f = 0.25;    %mass of ink cartidge in kg  (full).
Mc   = 0.018;     %mass of cable dragging cartidge in kg.
Jp   = 6.2e-6;  %pulley inertia in kg-m^2 (2 needed).
Js   = 1.4e-6;  %position sensor inertia in kg-m^2.
Rp   = 0.0065;    %pulley radii in m.
Gv   = 5;       %Voltage amplifier gain in V/V(control signal gain to motor)
Ks   = 25;    % position sensor gain in V/cm.
%in other words sensor produces 0V at starting position, 6.5V at the far
%end stopping position.

%% Motors
% Use engineering judgment and choose one of the following motors:

%Motor #1
Jm = 5e-5;    %Motor inertia in kg-m^2.
Bm = 3e-6;    %Motor viscous friction in Nm-s/rad.
Kt = 0.275;   %Motor torgue constant +/-10% Nm/A, (Kt = Ke.)
R  = 15;      %Motor resistance in ohms 
L  =35e-3;    %Motor inductance in mH.



%% Homework 
%Homework to do before the end of the semseter-number(4) in design sheet:
%b)From the signal flow graph apply Mason's Gain formula to find the
%analytical plant transfer function, G(s) =  Y(s)/U(s).  Using nominal
%component values, find the numerical plant transfer function coefficients
%and plant pole locations for all four motors.

%           --------     --------
%     + _   |      | u(t)|      |                    
% r(t)-|_|--| D(s) |-----| G(s) |--y(t)  
%      -|   |      |     |      | |                
%       |   --------     -------- |
%       ---------------------------              
%     
s = tf('s');

%The motor #1:
%the total inertia 
Jtotal = Jp*2 + Js + Jm + (Mi_e + Mc)*Rp^2

G = (Gv*Kt*Rp*Ks)/...
    (s^3*(L*Jtotal)+s^2*(R*Jtotal+Bm*L)+s*(Kt^2+R*Bm))


%ploting the zeros and poles of the plant of the first motor
figure(1);
pzmap(G);
title('zeros and poles of G(s) of the first motor');



%%  A unity feedback linear algebraic design controller - First Motor
%This method is used on the first motor.

%Inward Approach: finding the controller (D(s)) while having the closed
%loop transfer function and the plant (T(s) and G(s)).
%STEPSHAPE(Np,OS,Ts,SP) returns a continuous time system that meets the
% given unit step response specifications.

% Np :the poles and must be between 2 and 15
Np = 6;
% OS: percent overshoot and must be between 0 and 20
OS = 5;
% Ts is the settling time in seconds.
Ts = 0.01;
%SP is the settling time percentage. If SP is not given,
% SP = 2 is assumed.
SP = 2;

%the step response 
[No,Do]=stepshape(Np,OS,Ts,SP);

%Computes the compensator coefficients for the Unity Feedback.
% D is the controller in tf or zpk form.
% T is the closed loop transfer function in zpk form.
% Tu is the control effort transfer function in zpk form.
[D,T,Tu]=lamdesign(G,Do);
D = D
%Open loop 
%L_loop = D*G

%closed loop transfer function
%T = minreal(L_loop/(1 + L_loop))

%Plotting the closed transfer function 
figure(2);
step(T);
title('the closed TF using the first motor');

%Using the first motor, we are creating the input r(t) which is velocity vs
%position. 
%the function  dsigmf give the desired input velocity where the signal
%accelerates and then has constant velocity and after that deccelerates.
%the function dsigmf depends on four parameters;
%example; y = dsigmf(x,[a1 c1 a2 c2]).
%the slope at the begining
S1 = 10;
%the midpoint of slope
M1 = 0.3;
%The slope at the end
S2 = 18;
%The midpoint of the second slope
M2 = 25.5;
%the postion
Pos = linspace(0,27,5000); 
%Using dsigmf to find the velocity.  dsigmf is 
Vel = 66*dsigmf(Pos, [S1, M1, S2, M2]);
%plotting the velocity vs the position.
figure(3);
plot(Pos, Vel);
xlim([0 28]);
xlabel('Position in cm');
ylabel('Ink Cartridge Velocity cm/s');
title('Ink Cartridge Velocity vs Position of input');

%now we need the input to be in voltage vs the time, so we need to
%convert the velocity vs position plot to time using the function 
% cumtrapz(X,Y)which take cumulative integral with respect to X using
% trapezoidal integration.
time = cumtrapz(Pos, 1./Vel);
%position is converted to volts by multiplying by the position sensor gain
%which convert a position to volt.  Ks = 0.25% => 1/4
volt = Pos.*(1./4);
%plotting the voltage vs the time.
figure(4);
plot(time, volt);
xlim([0 0.7]);
xlabel('Time in ms');
ylabel('Voltage in v');
title('Voltage vs Time of input');

% %here is velocity vs time of the input 
% figure(14)
% plot(time, vel)
% xlim([0 0.7])
% xlabel('Time in ms');
% ylabel('velocity in cm/s');
% title('Velocity vs Time of input');

%here is position vs time of the input
figure(5);
plot(time, Pos);
xlim([0 0.7]);
xlabel('Time in s');
ylabel('position in cm');
title('position vs Time of input');

%spline(x,Y,xx) uses a cubic spline interpolation to find yy, the values 
%of the underlying function Y at the values of the interpolant xx. 
%For the interpolation, the independent variable is assumed to be the
%final dimension of Y with the breakpoints defined by x. The values in x 
%must be distinct.
 ti = linspace(0, .7, 5000);
%using spline will make the difference time equal which by then used in
%lsim.  
Volt1 = spline(time, volt, ti);
%the function Lsim will output the time response (Tres) and the output
%response Vres (voltage).
[Vres,Tres,y] = lsim(T,Volt1,ti);
%the output position is found by Vres/Ks.
X = Vres/.25; %in meter
%plotting the output position X
figure(6);
plot(Tres,X);
xlabel('Time in sec');
ylabel('Ink Position in cm');
title('Ink Position vs. Time');
% 
%Now we could find the velocity by taking the derivative of the position.
 Velocity = gradient(X)./gradient(Tres);
%Plotting the velocity vs time (Tres)
figure(7);
plot(Tres, Velocity);
xlabel('Time in sec');
ylabel('Ink Velocity in cm/s');
title('Ink Velocity vs. Time');
hline=refline([0 66]);
hline.Color = 'r';
set(hline,'LineStyle','-')

%Plotting Position vs velocity
figure(8);
plot(X, Velocity);
hline=refline([0 66]);
hline.Color = 'r';
set(hline,'LineStyle','-');
xlabel('Position in cm');
ylabel('Ink Velocity in cm/s');
title('Ink Velocity vs. Position');
xlim([0 28]);

%the acceleration is the derivative of velocity with time^2;
accele = diff(Velocity);
%plotting the acceleration vs time.
figure(9);
plot(Tres(1:end-1), accele);
xlabel('Time in s');
ylabel('Ink  Acceleration in cm/s^2');
title('Ink Acceleration vs. Time');

%Plotting acceleration vs position 
figure(10);
plot(X(1:end-1), accele);
xlabel('Position in cm');
ylabel('Ink Accelleration in cm/s^2');
title('Ink Acceleration vs. Position');
xlim([0 28]);

%finding the power vs time and vs position
%we found the transfer function for the current
Ti = (D*Gv*(Bm + Jtotal*s))/...
   ((R+L*s)*(Bm+Jtotal*s)+Kt^2+(D*Gv*Kt*Rp*Ks)/s);
I_output = lsim(Ti,Volt1,ti);

%plotting the power vs time
figure(11);
plot(Tres,I_output.^2.*R.*10^-2);
xlabel('Time in sec');
ylabel('power in mW');
title('power vs time');

%plotting power vs position
figure(12);
plot(Pos,I_output.^2.*R.*10^-2);
xlabel('position in cm');
ylabel('power in mW');
title('Power vs position');

%Plotting the velocity error vs position.
figure(13);
Velo_error = Velocity - transpose(Vel);
plot(Pos, Velo_error);
xlabel('Position in cm');
ylabel('Velocity in cm/s');
title('Velocity Error vs Position');

%Plotting the velocity error vs time.
figure(14);
plot(time, Velo_error);
xlabel('time in s');
ylabel('Velocity in cm/s');
title('Velocity Error vs time');


%plotting position error vs position 
Pos_error = X - transpose(Pos);
figure(15);
plot(Pos,X);
xlabel('Desired position in cm');
ylabel('position error');
title('position error vs Desired position');

%plotting position error vs time 
figure(16);
plot(time,Pos_error);
xlabel('time in s');
ylabel('position error');
title('position error vs time');

% %Findig the amplifier output voltage
Tvo = (s*(D*Gv*L*Jtotal)+(D*Gv*Bm*L))/...
    (L*s*Jtotal + R*Jtotal+ L*Bm + (Kt*Kt+R*Bm)/s + (D*Gv*Kt*Rp*Ks)/s^2);
V_output = lsim(Tvo,Volt1,ti);
V_output = V_output.*10^-1;
%plotting output voltage vs time
figure(17);
plot(Tres,V_output);
xlabel('Time in sec');
ylabel('Output voltage');
title('output voltage vs time');

figure(18);
plot(Pos,V_output);
xlabel('position in cm');
ylabel('Output voltage');
title('Output voltage vs position');
%% Cost of the project (First Motor)
%Peak motor power = 20*|P|:
Power_peak = 20 .* 7.03.*10^-3
%Number of controller parameters:
D = D       %System controller 
Nm = 10* 7  %the number of controller of parameters
%Number of specs not met:
Ns = 100*0
%completion date
Cd = 200.*max(0,0)
%total cost
Cost = Power_peak + Nm +Ns +Cd

%% Plotting plant , controller and overall closed loop.
%plotting the poles and zeros of each.

%plotting the plant pole locations
figure(19)
pzmap(G)
title('The plant pole locations')

%plotting the controller zero and pole locations
figure(20);
pzmap(D);
title('The controller zero and pole locations');

%plottign the closed loop transfer function zeros and poles
figure(21);
pzmap(T);
title('The closed loop transfer function');



