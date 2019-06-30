% symSandBox.m
% A symbolic manipulation sandbox for getting equations
clearvars; close all; clc;

syms fi th sy
% 3-2-1 sequence
R_sy_3 = getRmat(sy,3);
R_th_2 = getRmat(th,2);
R_fi_1 = getRmat(fi,1);
B_C_O = R_fi_1*R_th_2*R_sy_3;
syms qi ri wxi wyi wzi
omegaOBi = [0 qi ri].';
omegaBi1i = [wxi wyi wzi].';
cross(omegaOBi,omegaBi1i);



function rmat = getRmat(symAng,basDir)
    % Rotation Sequence from frame O to frame B
    % i.e. start in the O frame and rotate one angle to B frame
    switch basDir
        case 1
            rmat = [1 0 0;...
                0 cos(symAng) sin(symAng);...
                0 -sin(symAng) cos(symAng)];
        case 2
            rmat = [cos(symAng) 0 -sin(symAng);...
                0 1 0;...
                sin(symAng) 0 cos(symAng)];
        case 3
            rmat = [cos(symAng) sin(symAng) 0;...
                -sin(symAng) cos(symAng) 0;...                
                0 0 1];
        otherwise
            ME = error('getRmat only works in 3R');
            throw(ME);
    end
end % end getRmat