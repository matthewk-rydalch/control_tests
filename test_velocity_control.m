clear;
clc;

global x;
global pb pb_hist vb vb_hist;
global xc vc thc thdc;
global t tb tb_hist dt;
global Mt fe mu Jt;
global kpx kdx kpv kdv kpth kdth kpthd kdthd kff;
global vPrev thdPrev;

settleTime = 30;
tolerance = 0.03;
%[px,px_dot,th,th_dot]
x = [0,0,0,0];
pb = 1.0;
pb_hist = [];
vb = 1;
vb_hist = [];
xc = 0;
vc = 0;
thc = 0;
thdc = 0;
t = 0.0;
tb = 0;
tb_hist = [];
dt = 0.01;
Mt = 1.5;
fe = 9.81*Mt;
mu = 0.1; %Put this back on!!!!!!!!!
Jt = 0.0042+(2*0.5)*0.3^2;
kpx = .8;
kdx = 0;
kpv = 5;
kdv = 2;
kpth = 8;
kdth = 0.1;
kpthd = 10;
kdthd = 0;
kff = 0.0;%0.5*mu/Mt;
vPrev = 0;
thdPrev = 0;

f = 0;
tf = 10;
P = [Mt;fe;mu;Jt];
while t < tf
   boat_sim();
   tau = compute_control();
   dynamics(tau);
   t = t + dt;
%    plot_error()
   plot_positions()
   plot_velocity()
%     plot_attitude();
%     plot_thd()
end

% function plot_error()
%     global t x pb;
%     error = pb-x(1);
%     figure(1)
%     plot(t,error,'.r')
%     hold on
% end 

function plot_positions()
    global t x pb;
%     figure(2)
    plot(t,x(1),'.r')
    hold on
    plot(t,pb,'.b')
end

function plot_velocity()
    global t x vc vb
    plot(t,x(2),'.r')
    hold on
    plot(t,vb,'.b')
end

function plot_attitude()
    global t x thc
    plot(t,x(3),'.r')
    hold on
    plot(t,thc,'.b')
end

function plot_thd()
    global t x thdc
    plot(t,x(4),'.r')
    hold on
    plot(t,thdc,'.b')
end