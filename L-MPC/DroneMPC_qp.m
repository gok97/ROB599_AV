function control_input = DroneMPC_qp(A, B, parameters, initial_conditions, time_index)
    % extract constants
    horizon = parameters{1};
    Q = parameters{2};
    R = parameters{3};
    Xbar =  parameters{4};
    Ubar =  parameters{5};
    Xref =  parameters{6}(time_index:(time_index + horizon - 1), :);
    Uref =  parameters{7}(time_index:(time_index + horizon - 2), :);
    nx = parameters{8};
    nu = parameters{9};
    dt = parameters{10};
    K = parameters{11};

    %   Compute the entries to the lifted system matrix
    H = zeros(nx*horizon, nu*horizon);         %   Initialize H
    for i=1:horizon
        for j=1:i
            H(nx*(i-1)+1:nx*i, nu*(j-1)+1:nu*j) = A^(i-j)*B;
        end
    end

    %   Compute Q_bar
    Q_bar = zeros(nu*horizon);         %   Initialize Q
    for i=1:horizon
        Q_bar = Q_bar + H(nx*(i-1)+1:nx*i,:)'*Q*H(nx*(i-1)+1:nx*i,:);
    end
    Q_bar = Q_bar + R(1, 1)*eye(nu*horizon);

    %   Compute r
    r_bar = zeros(1, nu*horizon);      %   Initialize r
    for i=1:horizon
        r_bar = r_bar + 2*(initial_conditions-Xbar)*(A^i)'*Q*H(nx*(i-1)+1:nx*i,:);
    end

    % solve QP
    options = optimoptions('quadprog', 'Algorithm', 'active-set');
    [u_opt, J_opt, exitflag, output] = quadprog(2*Q_bar, r_bar);

    control_input = u_opt(1:nu)' + Uref(1, :);
end