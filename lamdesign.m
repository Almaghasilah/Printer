function [DF,TH,Tu]=lamdesign(varargin)
%Linear Algebraic Method Compensator Design.
%
%-- UNITY FEEDBACK DESIGN -------------------------------------------------
%
% [D,T,Tu]=LAMDESIGN(G,Do) computes the compensator coefficients
% for the Unity Feedback, Cascade Controller Configuration:
%
%           --------     --------
%     + _   |      | u(t)|      |               Y(s)         B(s)N(s)
% r(t)-|_|--| D(s) |-----| G(s) |--y(t)  T(s) = ---- = -------------------
%      -|   |      |     |      | |             R(s)   A(s)D(s) + B(s)N(s)
%       |   --------     -------- |
%       ---------------------------              U(s)
%                                        Tu(s) = ---- = Control Effort
%                                                R(s)
%
% G is the plant in tf or zpk form as defined by the Control Toolbox.
%
% Do is the desired denominator polynomial vector for the closed loop system T(s).
%
% If Do has as twice as many roots as G has poles, D(s) will be Type 1.
%
% If Do has one less than twice as many roots as G has poles, D(s) will be
% Type 0.
%
% D is the controller in tf or zpk form.
% T is the closed loop transfer function in zpk form.
% Tu is the control effort transfer function in zpk form.
%
%-- TWO PARAMETER FEEDBACK DESIGN -----------------------------------------
%
% [F,H,Tu]=LAMDESIGN(G,T,R) computes the compensator coefficients
% for the Two Parameter Controller Configuration:
%
%      --------          --------
%      |      |  + _ u(t)|      |                Y(s)         N(s)L(s)
% r(t)-| F(s) |---|_|----| G(s) |---y(t)  T(s) = ---- = -------------------
%      |      |   -|     |      | |              R(s)   A(s)D(s) + M(s)N(s)
%      --------    |     -------- |
%                  |     -------- |               U(s)
%                  |     |      | |       Tu(s) = ---- = Control Effort
%                  ------| H(s) |--               R(s)
%                        |      |
%                        --------
%
% G is the plant in tf or zpk form as defined by the Control Toolbox.
% T is the closed loop transfer function in tf or zpk form.
% T must have at least as many poles as the plant G.
% R is a vector of polynomial root locations that are chosen from in the
% design process on an as needed basis. These are "observer" poles that do
% not appear in T(s) or Tu(s).
%
% F is the feedforward controller in zpk form.
% H is the feedback controller in zpk form.
% Tu is the control effort transfer function zpk form.
%--------------------------------------------------------------------------
% General Notes:
% G and T must be STRICTLY proper
% Pole-Zero excess inequality must hold

% References:
% C.T. Chen, Linear System Theory and Design, 3rd ed., Oxford University
% Press, 1999, ISBN 0-19-511777-8.
%
% C.T. Chen and B. Seo, "The Inward Approach in the Design of Control
% Systems," IEEE Trans. on Education, vol. 33, no. 3, Aug. 1990, pp. 270-8.
%
% C.T. Chen, "Introduction to the Linear Algebraic Method for Control
% System Design," IEEE Control Systems Magazine, Oct. 1987, pp. 36-42.

% D.C. Hanselman, University of Maine, Orono, ME 04469
% Mastering MATLAB 7
% 2005-03-17, 2006-04-20, 2006-05-04
% 2008-03-28, 2009-03-24, 2013-04-25

msgid='LAMDESIGN:error';
%--------------------------------------------------------------------------
if nargin==2                      % LAMDESIGN(G,Do) Unity Feedback Design
  G = varargin{1};
  [N,D] = local_getTF(G);
	nd = length(D)-1;
  Do = varargin{2};
  if ~isvector(Do)
    error(msgid,'Do Must be a Polynomial Vector.');
  end
  ndo=length(Do)-1;
  if ndo==2*nd-1                % standard case, Type 0 A(s)
    S = local_silvester(N,D);
    p = S\Do(:);
    A = p(1:nd).';
    B = p(nd+1:end).';
  elseif ndo==2*nd              % make system Type 1 A(s)
    S = local_silvestor1(N,D);
    p = S\Do(:);
    A = [p(1:nd).' 0];
    B = p(nd+1:end).';
  elseif ndo<2*nd-1
    error(msgid,'Do must have at least %d more poles.',2*nd-1-ndo)
  elseif ndo>2*nd
    error(msgid,'Do must have at least %d fewer poles.',ndo-2*nd)
  end
  DF = zpk(tf(B,A)); % D(s)
  TH = zpk(tf(conv(B,N),Do)); % T(s)
  Tu = minreal(TH/G);  
   
elseif nargin==3              % LAMDESIGN(G,T,R) Two Parameter Design
  G = varargin{1};
  [N,D] = local_getTF(G);
  nd = length(D)-1;
  T = varargin{2};
  [No,Do] = local_getTF(T);
  if isnumeric(varargin{3}) && isreal(varargin{3}) && all(varargin{3}<0)
    R=varargin{3}(:);
  else
    error(msgid,'R Must be a Vector of Negative Real Root Locations.')
  end
  nr = length(R);
  
  Ntmp = local_minreal(N,No); % check retainment of non-min-phase-zeros
  if any(real(roots(Ntmp))>0)
    error(msgid,'T(s) Must Contain the Right Half Plane Zeros of G(s).')
  end
  [Np,Dp] = local_minreal(No,conv(Do,N));
  p = length(Dp)-1;
  if p < 2*nd-1
    r = 2*nd-1-p;
    if nr<r
      error(msgid,'%d More Roots Needed in R.',r-nr)
    end
    Dpb = poly(R(1:r));
  elseif p==2*nd-1
    Dpb = 1;
  else
    error(msgid,'T(s) not compatible with G(s).')
  end
  L = conv(Np,Dpb);
  S = local_silvester(N,D);
  F = conv(Dp,Dpb).';
  p = S\F;
  A = p(1:nd).';
  M = p(nd+1:end).';
  
  DF = zpk(tf(L,A)); % F(s)
  TH = zpk(tf(M,A)); % H(s)
  Tu = minreal(T/G);
else
   error(msgid,'Incorrect Number of Input Arguments.')
end
%--------------------------------------------------------------------------
function [num,den]=local_getTF(D)
msgid='LAMDESIGN:error';
if ~(isa(D,'tf')||isa(D,'zpk'))
  error(msgid,'Unknown Input.')
end
if isa(D,'zpk')
  D = tf(D);
end
num = get(D,'num');
num = num{1};
num = num(find(num,1):end); % strip leading zeros
den = get(D,'den');
den = den{1};
den = den(find(den,1):end);

if length(num)>=length(den)
  error(msgid,'System Must be Strictly Proper.')
end
%--------------------------------------------------------------------------
function S=local_silvester(N,D)
msgid='LAMDESIGN:error';
N=N(:);
nn=length(N);
D=D(:);
nd=length(D);
if nn<nd
   N=[zeros(nd-nn,1);N];
end
n=nd-1;
S=zeros(2*n);
for k=1:n
   S(k:k+n,k)=D;
   S(k:k+n,k+n)=N;
end
if condest(S)>1e13
   error(msgid,'Plant Must Not Have Any Uncanceled Equal Poles and Zeros.')
end
%--------------------------------------------------------------------------
function S=local_silvestor1(N,D)
msgid='LAMDESIGN:error';
N=N(:);
nn=length(N);
D=D(:);
nd=length(D);
if nn<nd
   N=[zeros(nd-nn,1);N];
end
n=nd-1;
S=zeros(2*nd-1);
for k=1:n
   S(k:k+n,k)=D;
   S(k:k+n,k+n)=N;
end
S(end-n:end,end)=N;
if condest(S)>1e13
   error(msgid,'Plant Must Not Have Any Uncanceled Equal Poles and Zeros.')
end
%--------------------------------------------------------------------------
function [nm,dm]=local_minreal(num,den)
tol=1e-5;
tol0=sqrt(eps);
z=roots(num);
p=roots(den);
nz=length(z);
zm=true(nz,1);
for i=1:nz
	if abs(z(i)>tol0)
		TOL=tol*abs(z(i));
	else
		TOL=tol;
	end
	match=find(abs(p-z(i))<=TOL);
	if ~isempty(match)
		p(match(1))=[]; % throw out matching pole
		zm(i)=false; % flag zero for elimination
	end
end
z=z(zm);
nm=real(num(1)*poly(z)/den(1));
dm=real(poly(p));