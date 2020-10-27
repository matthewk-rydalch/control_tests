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