clear;
clc;

global x;
global pb pb_hist vb vb_noise vb_hist;
global xc vc thc thdc;
global t tb tb_hist dt;
global Mt fe mu Jt;
global kpx kdx kix kpv kdv kpth kdth kpthd kdthd kff kffx;
global itermX;
global vPrev thdPrev;
global eLpf sigma;

%[px,px_dot,th,th_dot]
x = [0,0,0,0];
pb = .1;
pb_hist = [];
vb = 1;
vb_noise = vb;
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
kpx = 1.5;
kdx = 0.5;
kix = 1.0;
kpv = 5;
kdv = 2;
kpth = 8;
kdth = 0.1;
kpthd = 10;
kdthd = 0;
kffx = 1.0;
kff = 0.7*(mu/Mt+0.035);
itermX = 0;
vPrev = 0;
thdPrev = 0;
eLpf = xc-x(1);
sigma = 5;

f = 0;
tf = 25;
P = [Mt;fe;mu;Jt];
while t < tf
   boat_sim();
   tau = compute_control();
   dynamics(tau);
   t = t + dt;
%    plot_error()
   plot_positions()
%    plot_velocity()
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
    plot(t,vc,'.b')
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