function tau = compute_control()
    global xc vc thc thdc x t
    vc = compute_position_loop(xc);
%     vc = 1;
    ac = compute_velocity_loop(vc);
    thc = convert_2_theta(ac);
    thdc = compute_theta_loop(thc);
    tau = compute_theta_dot_loop(thdc);
end

function vc = compute_position_loop(xc)
    global x kpx kdx vb;
    px = x(1);
    e = xc-px;
    vc = e*kpx-x(2)*kdx+vb;
    vc = saturate(vc,-5.0,5.0);
end

function ac = compute_velocity_loop(vc)
    global x kpv kdv vPrev vb kff;
    v = x(2);
    a = differentiate_state(v,vPrev);
    e = vc-v;
    acMPerSSquared = e*kpv-a*kdv+vb*kff;
    acGs = acMPerSSquared/9.81;
    ac = saturate(acGs,-1,1);
    vPrev = v;
end

function thdc = compute_theta_loop(thc)
    global x kpth kdth;
    th = x(3);
    e = thc-th;
    thdc = e*kpth-x(4)*kdth;
%     thdc = saturate(thdc,-1.0,1.0);
end

function tau = compute_theta_dot_loop(thdc)
    global x kpthd kdthd thdPrev;
    thd = x(4);
    e = thdc-thd;
    thdd = differentiate_state(thd,thdPrev);
    tau = e*kpthd-thdd*kdthd;
    tau = saturate(tau,-1,1);
    thdPrev = thd;
end

function thc = convert_2_theta(ac)
    global Mt fe;
    %TODO fix this!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
    %acceleration in g's
%     thc = ac;%-asin(ac);
%     thc = ac*Mt/(-fe);
    thc = -asin(ac);
%     thc = saturate(thc,-0.2,0.2);
end

function u = saturate(u,uMin,uMax)
    if u > uMax
        u = uMax;
    elseif u < uMin
            u = uMin;
    end
end

function stateDot = differentiate_state(state,statePrev);
    global dt;
    stateDot = (state-statePrev)/dt;
end

    