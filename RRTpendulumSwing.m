clc; clear;
clf

bndry = [-3.14, 3.14, -10, 10]; % theta limits, theta dot limits (boundary)
dt = 0.02; % time step for each branch
N = 20000; % max number of samples

% Pendulum parameters
s.g = 9.81; % gravity
s.b = 0.1; % damping
s.max_torque = 7;

plot_c_space(bndry) % tree is in configuration (state) space

start = [-3.1415/2, 0, 0]; % start with pendulum down at rest
hold on
plot(start(1), start(2), 'r*')
goal = [3.1415/2, 0, 0]; % goal is swing up position
plot(goal(1), goal(2), 'r*')

PATH = generateRRT(start, goal, bndry, dt, s, N);
if ~isempty(PATH)
    plotPendulum(PATH(:, 1));
end

function plot_c_space(bndry)
figure(2);
rectangle('Position',[bndry(1), bndry(3), bndry(2) - bndry(1), ...
    bndry(4) - bndry(3)],'EdgeColor', 'k', 'LineWidth', 2);
axis([bndry])
set(gca,'XTick',-pi:pi/4:pi,'XTickLabel',{'-{\pi}','-3{\pi}/4','-{\pi}/2'...
     ,'-{\pi}/4','0','{\pi}/4','{\pi}/2','3{\pi}/4','{\pi}'});
set(gcf,'color','white')

xlabel('${\theta}\ (rad)$', 'Interpreter','latex');
ylabel('$\dot{\theta}\ (rad/sec)$', 'Interpreter','latex')

title('State-space of Pendulum')
hold on
end

function PATH = generateRRT(start, goal, bndry, dt, s, N)

config_space_limits = [bndry(1:2); bndry(3:4)]; % upper and lower limits
Q = make_random_points(config_space_limits, N);
assignin('base', 'Q', Q);
Vstart = start; % each vertex is a point in state space (q)
Estart = [];
figure(2);

tic
for i=1:min(size(Q,1), N)

    % search through config space
    if mod( i , 2000) == 0 % sample the goal every 2000 samples
        rand_q = goal(:,1:2);
    else
        rand_q = Q(i,:); % sample random point
    end

    % find nearest node in tree
    index1 = find_nearest_node(rand_q, Vstart(:,1:2));
    q1 = Vstart(index1, 1:2);

    new_node = extend(rand_q, q1, dt, s);

    % making plot continue on other side if it swings around
    beforeJump = new_node(1);
    if (new_node(1) > pi) || (new_node(1) < -pi)
        new_node(1) = mod(new_node(1)+pi, 2*pi) - pi;
    end

    % add new_node to tree
    Vstart = vertcat(Vstart, new_node);
    Estart = vertcat(Estart, [index1, length(Vstart)]);

    % plot new branch
    if abs(new_node(1) - beforeJump) < pi
        line([q1(1), new_node(1)],[q1(2), new_node(2)], 'Color', 'b');
    end

    % only plot every 500 iteration for speed
    if (i == 10) || (mod(i, 500)==0)
        drawnow;
    end
    
    % check for end condition ( 0.017 rad is 1 degree)
    if abs((new_node(:, 1) - goal(:, 1))) <= 0.07 && ...
            abs((new_node(:, 2) - goal(:, 2))) <= 0.5
        break
    end
end

% pick the best path in V and E
PATH = pick_path(Estart, Vstart, goal);

% Plotting final path
figure(2);
for i = 2:size(PATH, 1)
    if norm(PATH(i-1,1:2) - PATH(i,1:2)) <= 1
        line([PATH(i-1,1), PATH(i,1)],[PATH(i-1,2), PATH(i,2)],'Color','r',...
            'LineWidth', 2);
    end
end

hold off

toc
end

function Q = make_random_points(config_space_limits, k)
% k is number of random points
% config_space_limits is [[lowerlim, upperlim]; [lowerlim, upperlim]] 

L_lim1 = config_space_limits(1, 1);
U_lim1 = config_space_limits(1, 2);
L_lim2 = config_space_limits(2, 1);
U_lim2 = config_space_limits(2, 2);

x = randi([L_lim1 * k, U_lim1 * k], [1, k]) / k;
y = randi([L_lim2 * k, U_lim2 * k], [1, k]) / k;
Q(:, 1) = x;
Q(:, 2) = y;
end

function closest_node_idx = find_nearest_node(q_rand, V)
q = repmat(q_rand, size(V,1), 1);
d = sqrt(sum((V-q).^2,2));
[~, closest_node_idx] = min(d);
end

function new_node = extend(rand_q, q, dt, s)
% interpolate path  for set time and check for collision in each config waypoint

olddist = 1e10;
for i = linspace(-s.max_torque,s.max_torque,10)
    xdot = dynamics(q,i,s);
    xShift = q + dt*xdot;
    dist = norm(xShift' - rand_q);
    
    if dist < olddist
        bestx = xShift; % with torque that would extend out closest to rand q
        bestTorque = i;
        olddist = dist;
    end
end

theta = bestx(1);
theta_dot = bestx(2);

new_node = [theta, theta_dot, bestTorque];
end

function qdot = dynamics(q, u, s)    
    % qdot = [theta dot, theta double dot]
	qdot = [q(2), (u - s.g*cos(q(1))-s.b*q(2))]; 
end

function best_path = pick_path(Estart, Vstart, goal)
start_path = [];

if norm(Vstart(end,1:2) - goal(:, 1:2)) <= 0.5 % if tree reached goal
    row = size(Vstart, 1);
    for z = 1:size(Estart, 1)
        start_path = vertcat(start_path, Vstart(row, :) );
        next_row = Estart(row-1, 1);

        if next_row == 1
            break
        else
            row = next_row;
        end
    end

    % Combine and reorient
    start_path = vertcat(start_path, Vstart(1, :));
    best_path = flip(start_path);
    
else  % the path did not connect to goal
    best_path = [];
    disp('path did not connect to goal')
end
end

function plotPendulum(thetas)
figure(1);
clf
bndry = [-1.5, 1.5, -1.5, 1.5];
rectangle('Position',[bndry(1), bndry(3), bndry(2) - bndry(1), ...
    bndry(4) - bndry(3)],'EdgeColor', 'k', 'LineWidth', 2);
axis(bndry);
axis off

set(gcf,'color','white')
title('Pendulum Swing-up Path')
hold on

plot_w_space([-1.5, 1.5, -1.5, 1.5]); hold on

xL = cos(thetas);
yL = sin(thetas);

for i = 1:(length(thetas) + 30)
    if i < length(thetas)
        x = cos(thetas(i));
        y = sin(thetas(i));
    end
    rectangle('Position', [bndry(1), bndry(3), bndry(2) - bndry(1), ...
    bndry(4) - bndry(3)], 'FaceColor', [1 1 1 0.03]);
    rectangle('Position', [-0.05 -0.05 0.1 0.1], 'FaceColor', [1 0 1]);
    plot(x, y,'o','Color',[0 0 0]);
    plot([0 x],[0 y],'Color', [0 0 0], 'LineWidth',1.6);
    drawnow
end

end

function plot_w_space(bndry)
figure(1)
rectangle('Position',[bndry(1), bndry(3), bndry(2) - bndry(1), ...
    bndry(4) - bndry(3)],'EdgeColor', 'k', 'LineWidth', 2);
axis([bndry])
end


