% Function to execute the simulation of the quadcopter using a Non-Linear Time-Varying MPC Controller
function [actualState, actualControl, endTime] = nlmpc_simulator(desiredState, referenceControl, matrixWind, matrixMass, controlConstants, modelConstants)

    % Extract constants to make the function more legible
    dt = controlConstants{1};
    tTot = (length(desiredState)-1)*dt;
    pw = controlConstants{2}; 
    hz = controlConstants{3};
    controlLimits = controlConstants{10};
    controlPenalty = controlConstants{4};
    controlRatePenalty = controlConstants{5};
    stateWeights = controlConstants{9};
    
    % Define a nominal control target to maintain the quadcopter hovering/stable
    nloptions = nlmpcmoveopt;
    nloptions.MVTarget = referenceControl;
    mv = nloptions.MVTarget;
    
    % Display a waitbar to show the simulation progress
    hbar = waitbar(0,"Simulation Progress");
    
    % Initialize the controller at the set target
    lastMV = mv;
    
    % Initialise the arrays tracking the state and controls history
    actualState = desiredState(1, :);
    actualControl = lastMV;
    
    % Simulation loop
    tic
    for k = 1:(tTot/dt)
        
        % Read the trajectory
        yref = safe_reference(k, k+pw-1, desiredState);
        
        % If the mass or wind change, the model has effectively changed and new state functions and jacobians need to be recomputed               
        if k == 1 || (matrixMass(k-1, 1) ~= matrixMass(k, 1) || matrixWind(k, 1) ~= matrixWind(k-1, 1))

            % Substitute the new constants
            modelConstants(1) = matrixMass(k, 2);
            modelConstants(2) = matrixMass(k, 3);
            modelConstants(3) = matrixMass(k, 4);
            modelConstants(10) = matrixWind(k, 1);
            modelConstants(11) = matrixWind(k, 2);
            modelConstants(12) = matrixWind(k, 3);
            modelConstants(17) = matrixMass(k, 1);
    
            % Generate the state functions and jacobian functions
            QuadcopterModel;
    
            % Define the non-linear mpc problem
            nx = controlConstants{6};
            ny = controlConstants{7};
            nu = controlConstants{8};
            nlmpcobj = nlmpc(nx, ny, nu);
            
            % Associate the state and the jacobian functions with the problem
            nlmpcobj.Model.StateFcn = "QuadrotorStateFcn";
            nlmpcobj.Jacobian.StateFcn = @QuadrotorStateJacobianFcn;

            % Validate as necessary
            rng(0)
            validateFcns(nlmpcobj,rand(nx,1),rand(nu,1));
            
            % Specify the MPC parameters
            nlmpcobj.Ts = dt;
            nlmpcobj.PredictionHorizon = pw;
            nlmpcobj.ControlHorizon = hz;
            
            % Add constraints to the control inputs:
            nlmpcobj.MV = controlLimits;
            
            % Define the weights for the non-linear MPC
            nlmpcobj.Weights.OutputVariables = stateWeights;
            nlmpcobj.Weights.ManipulatedVariables = [controlPenalty controlPenalty controlPenalty controlPenalty];
            nlmpcobj.Weights.ManipulatedVariablesRate = [controlRatePenalty controlRatePenalty controlRatePenalty controlRatePenalty];
        end
        
        % Compute the control move with reference previewing
        xk = actualState(k,:);
        [uk,nloptions,~] = nlmpcmove(nlmpcobj,xk,double(lastMV),yref,[],nloptions);
    
        % Store the control move
        actualControl(k+1,:) = uk';
        lastMV = uk;
    
        % Simulate the quadrotor for the next control interval (MVs = uk) 
        ODEFUN = @(t,xk) QuadrotorStateFcn(xk,uk);
        [~,XOUT] = ode45(ODEFUN,[0 dt], actualState(k,:)');
    
        % Update the quadrotor state
        actualState(k+1,:) = XOUT(end,:);
    
        % Update the waitbar and print the status
        waitbar(k*dt/tTot,hbar);
        disp("Current Iteration: " + k +" out of " + length(desiredState))
    end
    
    % Store the total simulation time
    endTime = toc;
    
    % Close the waitbar 
    close(hbar)

end