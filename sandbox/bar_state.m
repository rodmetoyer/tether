function xdot = bar_state(t,x,thr)
% Computes the state derivative for a bar-model tether

numlinks = thr.numlinks;
L = thr.baseMoment; % start of the 3N+3 X 1 link moments vector | goes with CL, the only 3N X 3N+3 matrix
% Loop over links and build matrices
for i=1:1:numlinks
    % NOTATION NOTE: k = i-1
    % Normalize Euler Parameters 
    % q0j = x(7*i-3) q1j = x(7*i-2) q2j = x(7*i-1) q3j = x(7*i)
    qj = [x(7*i-3);x(7*i-2);x(7*i-1);x(7*i)];
    qj = qj/norm(qj);
    % Get omega j ready to use [p q r]
    omegaj = [x(7*i-6); x(7*i-5); x(7*i-4)];
    omegajX = [0 -omegaj(3) omegaj(2);...
               omegaj(3) 0 -omegaj(1);...
               -omegaj(2) omegaj(1) 0];
    j_C_O = [qj(1)^2+qj(2)^2-qj(3)^2-qj(4)^2, 2*(qj(2)*qj(3) + qj(1)*qj(4)),   2*(qj(2)*qj(4) - qj(1)*qj(3));...
             2*(qj(2)*qj(3) - qj(1)*qj(4)),   qj(1)^2-qj(2)^2+qj(3)^2-qj(4)^2, 2*(qj(3)*qj(4) + qj(1)*qj(2));...
             2*(qj(2)*qj(4) + qj(1)*qj(3)),   2*(qj(3)*qj(4) - qj(1)*qj(2)),   qj(1)^2-qj(2)^2-qj(3)^2+qj(4)^2];
    CF(3*i-2:3*i,3*i-2:3*i) = j_C_O;  % puts rotation matices on the diagonal
    CA(3*i-2:3*i,3*i-2:3*i) = eye(3); % puts identity matices on the diagonal
    rjX = [0 0 0; 0 0 -thr.link(i).x; 0 thr.link(i).x 0;];
    T(3*i-2:3*i,3*i-2:3*i) = rjX;
    if i == 1
        velLinkCM = omegajX*[thr.link(i).x;0;0];
        velLinkCMprevious = velLinkCM;
        j_C_k = j_C_O;
        ST(3*i-2:3*i,1) = -omegajX*(omegajX*[thr.link(i).x;0;0]); % ST is 3Nx1 vector
    else
        omegajminus = [x(7*(i-1)-6); x(7*(i-1)-5); x(7*(i-1)-4)];
        omegajXminus = [0 -omegajminus(3) omegajminus(2);...
               omegajminus(3) 0 -omegajminus(1);...
               -omegajminus(2) omegajminus(1) 0];
        velLinkEndprevious = velLinkCMprevious + omegajXminus*[thr.link(i-1).c;0;0];
        velLinkCM = velLinkEndprevious + omegajX*[thr.link(i).x;0;0];
        velLinkCMprevious = velLinkCM;
        qj = [x(7*(i-1)-3) x(7*(i-1)-2) x(7*(i-1)-1) x(7*(i-1))];
        k_C_O = [qj(1)^2+qj(2)^2-qj(3)^2-qj(4)^2, 2*(qj(2)*qj(3) + qj(1)*qj(4)),   2*(qj(2)*qj(4) - qj(1)*qj(3));...
                 2*(qj(2)*qj(3) - qj(1)*qj(4)),   qj(1)^2-qj(2)^2+qj(3)^2-qj(4)^2, 2*(qj(3)*qj(4) + qj(1)*qj(2));...
                 2*(qj(2)*qj(4) + qj(1)*qj(3)),   2*(qj(3)*qj(4) - qj(1)*qj(2)),   qj(1)^2-qj(2)^2-qj(3)^2+qj(4)^2];
        j_C_k = j_C_O*transpose(k_C_O);
        if i < thr.numlinks
            CA(3*i+1:3*i+3,3*i-2:3*i) = -j_C_k; % Adds j_C_k to diagonal edge of the lower triangle - everything else is 0
            T(3*i+1:3*i+3,3*i-2:3*i) = j_C_k*rjX;        
        end        
        ST(3*i-2:3*i,1) = -j_C_k*omegajXminus*(omegajXminus*[thr.link(i-1).c;0;0]) - omegajX*(omegajX*[thr.link(i).x;0;0]);
    end    
    I(3*i-2:3*i,3*i-2:3*i) = thr.link(i).inertia; % puts the inertia matrix on the diagonal - everything else is 0    
    SR(3*i-2:3*i,3*i-2:3*i) = -rjX*j_C_k; % puts the -rjX*j_C_k product on the diagonal
    CR(3*i-2:3*i,3*i-2:3*i) = j_C_k; % puts j_C_k on the diagonal
    CL(3*i-2:3*i,3*i-2:3*i) = j_C_k; % puts j_C_k on the diagonal
    CL(3*i-2:3*i,3*i+1:3*i+3) = -eye(3); % Adds rjX to diagonal edge of the upper triangle to finish with 3N X 3N+3 matrix
    M(3*i-2:3*i,3*i-2:3*i) = thr.link(i).mass*eye(3);
    if i < thr.numlinks
        SR(3*i-2:3*i,3*i+1:3*i+3) = rjX; % Adds rjX to diagonal edge of the upper triangle - everything else is 0
        CR(3*i-2:3*i,3*i+1:3*i+3) = -eye(3);
    end    
    bigOmegaX(4*i-3:4*i,4*i-3:4*i) = [0 -omegaj(1) -omegaj(2) -omegaj(3);...
                                      omegaj(1) 0 omegaj(3) -omegaj(2);...
                                      -omegaj(2) -omegaj(3) 0 omegaj(1);...
                                      omegaj(3) omegaj(2) -omegaj(1) 0];
    % Build last of the matrices. The ones that only have stuff at the end
    SF(3*i-2:3*i,3*i-2:3*i) = zeros;
    FL(3*i-2:3*i,1) = [0;0;0];
    L(3*i+1:3*i+3,1) = thr.link(i).moment;
    Somega(3*i-2:3*i,1) = omegajX*(thr.link(i).inertia*omegaj);
    FD = computeExternalForces(transpose(j_C_O)*velLinkCM,j_C_O);
    FT(3*i-2:3*i,1) = FD + [0;0;thr.link(i).weight];
    if i==thr.numlinks
        SF(3*i-2:3*i,3*i-2:3*i) = rjX*j_C_O;
        FL(3*i-2:3*i,1) = thr.endForce;
        FT(3*i-2:3*i,1) = FT(3*i-2:3*i,1) + thr.endForce;
    end    
end % end links loop
disp('test complete');
% i is now whatever the number of links is so just use that
xdot = NaN(7*i,1);
CRinvM = CR\M;
CAinvT = CA\T;
temp1 = I+SR*CRinvM*CAinvT;
temp2 = SF*FL + CL*L - Somega;
CAinvST = CA\ST;
temp3 = M*CAinvST+CF*FT;
CRinvtemp3 = CR\temp3;
temp4 = temp2-SR*CRinvtemp3;
omegadot = temp1\temp4;
xdot(7*i-3:7*i) = bigOmegaX*x(7*i-3:7*i);
end % end bar_state

function f = computeExternalForces(velocity,orientation)
    % Computes external hydrodynamic forces on a rigid link
    % todo(rodney) make this depend on both velocity and orientation
    % todo(rodney) make this a method in the rigid link class
    % todo(rodney) add environment argument to get flow speed
    tempFlowSpeed = [1;0;0];
    relVel = velocity - tempFlowSpeed;
    shittyViscosityCoefficent = 0.25;
    f = -shittyViscosityCoefficent*relVel;
end % end computeExternalForces