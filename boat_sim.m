function boat_sim()
    global t dt xc pb vb
    pb = pb+vb*dt;
    xc = pb;
end