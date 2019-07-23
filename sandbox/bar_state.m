function xdot = bar_state(t,x,thr)
% Computes the state derivative for a bar-model tether



numlinks = thr.numlinks;
L = thr.baseMoment; % start of the 3N+3 X 1 link moments vector
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
    CA(3*i-2:3*i,3*i-2:3*i) = eye(3); % puts identity matices on the diagonal
    rjX = [0 0 0; 0 0 -thr.link(i).x; 0 thr.link(i).x 0;];
    T(3*i-2:3*i,3*i-2:3*i) = rjX;
    if i == 1
        j_C_k = j_C_O;
    else
        qj = [x(7*(i-1)-3) x(7*(i-1)-2) x(7*(i-1)-1) x(7*(i-1))];
        k_C_O = [qj(1)^2+qj(2)^2-qj(3)^2-qj(4)^2, 2*(qj(2)*qj(3) + qj(1)*qj(4)),   2*(qj(2)*qj(4) - qj(1)*qj(3));...
                 2*(qj(2)*qj(3) - qj(1)*qj(4)),   qj(1)^2-qj(2)^2+qj(3)^2-qj(4)^2, 2*(qj(3)*qj(4) + qj(1)*qj(2));...
                 2*(qj(2)*qj(4) + qj(1)*qj(3)),   2*(qj(3)*qj(4) - qj(1)*qj(2)),   qj(1)^2-qj(2)^2-qj(3)^2+qj(4)^2];
        j_C_k = j_C_O*transpose(k_C_O);
        if i < thr.numlinks
            CA(3*i+1:3*i+3,3*i-2:3*i) = -j_C_k; % Adds j_C_k to diagonal edge of the lower triangle - everything else is 0
            T(3*i+1:3*i+3,3*i-2:3*i) = j_C_k*rjX;
        end
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
        
    % Build last of the matrices. The ones that only have stuff at the end
    SF(3*i-2:3*i,3*i-2:3*i) = zeros;
    FL(3*i-2:3*i,1) = [0;0;0];
    L(3*i+1:3*i+3,1) = thr.link(i).moment;
    Somega(3*i-2:3*i,1) = omegajX*(thr.link(i).inertia*omegaj);
    if i==thr.numlinks
        SF(3*i-2:3*i,3*i-2:3*i) = rjX*j_C_O;
        FL(3*i-2:3*i,1) = thr.endForce;
    end
    
end % end links loop
disp('test complete');
end % end bar_state
