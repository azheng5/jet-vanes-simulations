% Initial euler angle - ZYX convention
% phi = z rotation = yaw
% theta = y rotation = pitch
% psi = x rotation = roll

v_a1 = [1;0;0];
bigPhi_a2a1 = [0 5*(pi/180) 3*(pi/180)];
v_a2 = eulerRotate(v_a1, bigPhi_a2a1)
bigPhi_a3a2 = [0 0 pi/2];
v_a3 = eulerRotate(v_a2,bigPhi_a3a2)


% % validate with matlab
% rotm = eul2rotm(bigPhi);
% v_f_true = rotm*v_i

v_b1 = v_a1;
bigPhi_b2b1 = [0 3*(pi/180) 5*(pi/180)];
v_b2 = eulerRotate(v_b1, bigPhi_b2b1)



function v_f = eulerRotate(v_i,bigPhi)
    % v_i (3x1) vector in initial frame
    % v_f (3x1) vector in final frame
    % bigPhi (1x3) ZYX euler angles from initial to final frame

    % define euler angles
    psi = bigPhi(1);
    tht = bigPhi(2);
    phi = bigPhi(3);

    % rotn matrix from initial frame i to final frame f
    Cfi = [cos(tht)*cos(phi), cos(phi)*sin(psi)*sin(tht) - cos(psi)*sin(phi), sin(psi)*sin(phi) + cos(psi)*cos(phi)*sin(tht);
       cos(tht)*sin(phi), cos(psi)*cos(phi) + sin(psi)*sin(tht)*sin(phi), cos(psi)*sin(tht)*sin(phi) - cos(phi)*sin(psi);
       -sin(tht),        cos(tht)*sin(psi),                           cos(psi)*cos(tht)];

    v_f = Cfi*v_i;
end