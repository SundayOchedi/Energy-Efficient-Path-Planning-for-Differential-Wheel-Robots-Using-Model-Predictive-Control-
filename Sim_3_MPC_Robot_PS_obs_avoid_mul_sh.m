function Sim_3_MPC_Robot_PS_obs_avoid_mul_sh(x_track,y_track,theta_track,map)
% CasADi v3.4.5
addpath('casadi-3.6.5-windows64-matlab2018b')
import casadi.*

T = 0.1; %[s]
N = 10; % prediction horizon
rob_diam = 0.3;

% Define velocity square limits
V_min_sq = 0.25;  % Minimum squared velocity(Kinetic Energy)
V_max_sq = 0.6;  % Maximum squared velocity(Kinetic Energy)

v_max = 2; v_min = -v_max;
omega_max = pi/4; omega_min = -omega_max;

x = SX.sym('x'); y = SX.sym('y'); theta = SX.sym('theta');
states = [x;y;theta]; n_states = length(states);

v = SX.sym('v'); omega = SX.sym('omega');
controls = [v;omega]; n_controls = length(controls);
rhs = [v*cos(theta);v*sin(theta);omega]; % system r.h.s

f = Function('f',{states,controls},{rhs}); % nonlinear mapping function f(x,u)
U = SX.sym('U',n_controls,N); % Decision variables (controls)
P = SX.sym('P',n_states + n_states);
% parameters (which include at the initial state of the robot and the reference state)

X = SX.sym('X',n_states,(N+1));
% A vector that represents the states over the optimization problem.

obj = 0; % Objective function
g = [];  % constraints vector

Q = zeros(3,3); Q(1,1) = 1.5;Q(2,2) = 5;Q(3,3) = 0.1; % weighing matrices (states)
R = zeros(2,2); R(1,1) = 0.5; R(2,2) = 0.05; % weighing matrices (controls)

st  = X(:,1); % initial state
g = [g;st-P(1:3)]; % initial condition constraints
for k = 1:N
    st = X(:,k);  con = U(:,k);
    obj = obj+(st-P(4:6))'*Q*(st-P(4:6)) + con'*R*con; % calculate obj
    st_next = X(:,k+1);
    f_value = f(st,con);
    st_next_euler = st+ (T*f_value);
    g = [g;st_next-st_next_euler]; % compute constraints

    % % Kinetic energy constraints
    % % Ensure that squared velocity is within specified bounds
    % g = [g; con(1)^2 - V_max_sq]; % Upper bound for squared velocity
    % g = [g; V_min_sq - con(1)^2];  % Lower bound for squared velocity

end
% Add constraints for collision avoidance
obs_x = 18; % meters
obs_y = 20; % meters
obs_diam = 4; % meters
for k = 1:N+1   % box constraints due to the map margins
    g = [g ; -sqrt((P(4)-obs_x)^2+(P(5)-obs_y)^2) + (obs_diam/2 + obs_diam/2)];
end
% make the decision variable one column  vector
OPT_variables = [reshape(X,3*(N+1),1);reshape(U,2*N,1)];

nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 100;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);

args = struct;
args.lbg(1:3*(N+1)) = 0; % equality constraints
args.ubg(1:3*(N+1)) = 0; % equality constraints

args.lbg(3*(N+1)+1 : 3*(N+1)+ (N+1)) = -inf; % inequality constraints
args.ubg(3*(N+1)+1 : 3*(N+1)+ (N+1)) = 0; % inequality constraints

% % Set up bounds for the kinetic energy constraints for each control interval
% additional_constraints = 2 * N; % two new constraints (upper and lower) per control interval
% total_constraints = numel(g); % Update total number of constraints after adding kinetic energy constraints
% args.lbg(total_constraints-additional_constraints+1:2:total_constraints) = -10; % Lower bounds for kinetic energy constraints
% args.ubg(total_constraints-additional_constraints+1:2:total_constraints) = 0;    % Upper bounds for V_max_sq constraint
% args.lbg(total_constraints-additional_constraints+2:2:total_constraints) = 0;     % Lower bounds for V_min_sq constraint
% args.ubg(total_constraints-additional_constraints+2:2:total_constraints) = 10;   % Upper bounds for kinetic energy constraints


args.lbx(1:3:3*(N+1),1) = 0; %state x lower bound
args.ubx(1:3:3*(N+1),1) = 50; %state x upper bound
args.lbx(2:3:3*(N+1),1) = 0; %state y lower bound
args.ubx(2:3:3*(N+1),1) = 50; %state y upper bound
args.lbx(3:3:3*(N+1),1) = -inf; %state theta lower bound
args.ubx(3:3:3*(N+1),1) = inf; %state theta upper bound

args.lbx(3*(N+1)+1:2:3*(N+1)+2*N,1) = v_min; %v lower bound
args.ubx(3*(N+1)+1:2:3*(N+1)+2*N,1) = v_max; %v upper bound
args.lbx(3*(N+1)+2:2:3*(N+1)+2*N,1) = omega_min; %omega lower bound
args.ubx(3*(N+1)+2:2:3*(N+1)+2*N,1) = omega_max; %omega upper bound
%----------------------------------------------
% ALL OF THE ABOVE IS JUST A PROBLEM SET UP


% THE SIMULATION LOOP SHOULD START FROM HERE
%-------------------------------------------

%reference to track
new_xs_x=x_track;
new_xs_y=y_track;
new_xs_theta=theta_track;


t0 = 0;
%x0 = [0 ; 0 ; 0.0];    % initial condition.
x0 = [new_xs_x(1) ; new_xs_y(1) ; new_xs_theta(1)]; % initial condition.
%xs = [0 ; 0 ; 0]; % Reference posture.
xs = [new_xs_x(2) ; new_xs_y(2) ; new_xs_theta(2)]; % Reference posture.

xx(:,1) = x0; % xx contains the history of states
t(1) = t0;

u0 = zeros(N,2);        % two control inputs for each robot
X0 = repmat(x0,1,N+1)'; % initialization of the states decision variables

sim_tim = 80; % Maximum simulation time

% Start MPC
mpciter = 0;
xx1 = [];
u_cl=[];

% Initialization of error storage arrays
x_errors = [];
y_errors = [];

% the main simulaton loop... it works as long as the error is greater
% than 10^-6 and the number of mpc steps is less than its maximum
% value.
main_loop = tic;
while(norm((x0-xs),2) > 1e-2 && mpciter < sim_tim / T)
    args.p   = [x0;xs]; % set the values of the parameters vector
    % initial value of the optimization variables
    args.x0  = [reshape(X0',3*(N+1),1);reshape(u0',2*N,1)];
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
        'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
    u = reshape(full(sol.x(3*(N+1)+1:end))',2,N)'; % get controls only from the solution
    xx1(:,1:3,mpciter+1)= reshape(full(sol.x(1:3*(N+1)))',3,N+1)'; % get solution TRAJECTORY
    u_cl= [u_cl ; u(1,:)];
    t(mpciter+1) = t0;
    % Apply the control and shift the solution
    [t0, x0, u0] = shift(T, t0, x0, u,f);
    xx(:,mpciter+2) = x0;
    X0 = reshape(full(sol.x(1:3*(N+1)))',3,N+1)'; % get solution TRAJECTORY
    % Shift trajectory to initialize the next step
    X0 = [X0(2:end,:);X0(end,:)];
     % Update reference posture for the next iteration
    if mpciter+1 <= length(new_xs_x)
        xs = [new_xs_x(mpciter+1); new_xs_y(mpciter+1); new_xs_theta(mpciter+1)];
    end
    mpciter;
    mpciter = mpciter + 1;
    % Calculate and store the position errors
    x_errors(mpciter) = x0(1) - xs(1);
    y_errors(mpciter) = x0(2) - xs(2);
end;
main_loop_time = toc(main_loop);
ss_error = norm((x0-xs),2);
average_mpc_time = main_loop_time/(mpciter+1);


Draw_MPC_point_stabilization_v1 (t,xx,xx1,u_cl,xs,N,rob_diam,new_xs_x,new_xs_y,new_xs_theta,map); % a drawing function
errorplot(t,x_errors,y_errors,x0);
end

