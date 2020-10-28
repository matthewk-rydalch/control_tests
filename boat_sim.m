function boat_sim()
    global t dt xc pb vb vb_noise
    vb_noise = sin(t)+vb;
    pb = pb+vb_noise*dt;
    xc = pb;
end