function dynamics(tau)
    global x Jt Mt fe mu xdd;
    thdd = 1/Jt*tau;
    x(4) = integrate_state(x(4),thdd);
    x(3) = integrate_state(x(3),x(4));
    xdd = 1/Mt*(-fe*x(3))-mu*x(2);
    x(2) = integrate_state(x(2),xdd);
    x(1) = integrate_state(x(1),x(2));
end

function xOld = integrate_state(xOld,xdot)
    global dt;
    xOld = xOld + xdot*dt;
end

% function a = newtons_law(f)
%     global Mt;
%     a = f/Mt;
% end
% function xd=dynamics(t,x,P)
%     xd = zeros(2,1);
%     
%     Mt = P(1);
%     fe = P(2);
%     mu = P(3);
%     Jt = P(4);
%     px = x(1);
%     pxd = x(2);
% %     th = x(3);
% %     thd = x(4);
%     
%     get_dt(t);
%     boat_sim(t);
%     tau = compute_control(x);
%     
%     xd(1) = pxd;
%     xd(2) = 1/Mt*tau;
% %     xd(1) = pxd;
% %     xd(2) = -1/Mt*tau;%1/Mt*(-fe*th)-mu*pxd;
% %     xd(3) = thd;
% %     xd(4) = 1/Jt*tau;
% 
% end

function get_dt(tNow)
    global t dt
    dt = tNow-t;
    if dt < 0
        dt = 0;
    end
%     if dt > 0.1
%         a = 1;
%     end
    t = tNow;
end