format compact
set(0,'DefaultFigureWindowStyle','docked')

%% Read files
t0_file = fopen('../build/quadrotor_init_t.dat', 'r');
x0_file = fopen('../build/quadrotor_init_x.dat', 'r');
u0_file = fopen('../build/quadrotor_init_u.dat', 'r');

tf_file = fopen('../build/quadrotor_final_t.dat', 'r');
xf_file = fopen('../build/quadrotor_final_x.dat', 'r');
uf_file = fopen('../build/quadrotor_final_u.dat', 'r');


t0 = fread(t0_file, 'double');
x0 = reshape(fread(x0_file, 'double'), 6, []);
u0 = reshape(fread(u0_file, 'double'), 5, []);

tf = fread(tf_file, 'double');
xf = reshape(fread(xf_file, 'double'), 6, []);
uf = reshape(fread(uf_file, 'double'), 5, []);

x_names = ["px", "py", "pz", "vx", "vy", "vz"];
u_names = ["qw", "qx", "qy", "qz", "F"];

%% Plot position
figure(1); clf;
set(gcf,'color','w');
set(gcf, 'name', 'Position', 'NumberTitle', 'off');
for i = 1:3
    idx = i;
    subplot(3,1,i);
    plot(t0, x0(idx,:));
    hold on;
    plot(tf, xf(idx,:));
    title(x_names(idx));
    legend("x0", "xf")
end

%% Plot vel
figure(2); clf;
set(gcf,'color','w');
set(gcf, 'name', 'Velocity', 'NumberTitle', 'off');
for i = 1:3
    idx = i+3;
    subplot(3,1,i);
    plot(t0, x0(idx,:));
    hold on;
    plot(tf, xf(idx,:));
    title(x_names(idx));
    legend("x0", "xf")
end


%% Plot Inputs
figure(3); clf;
set(gcf,'color','w');
set(gcf, 'name', 'Inputs', 'NumberTitle', 'off');
for i = 1:5
    idx = i;
    subplot(5,1,i);
    plot(t0, u0(idx,:));
    hold on;
    plot(tf, uf(idx,:));
    title(u_names(idx));
    legend("x0", "xf")
end

%% Plot 3d
figure(4); clf;
set(gcf, 'name', '3D Trajectory', 'NumberTitle', 'off');
plot3(x0(1,:), x0(2,:), x0(3,:), '--', 'linewidth', 2.0); hold on;
plot3(xf(1,:), xf(2,:), xf(3,:));
legend("x0", "xf")