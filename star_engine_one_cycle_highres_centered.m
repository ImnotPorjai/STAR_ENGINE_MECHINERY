function star_engine_one_cycle_highres_centered()

    %% Load engine parameters
    [n, r, l, phi, ~, ~, ~, ~, ~, base_speed] = engine_params();

    %% Symbolic setup
    syms theta_sym
    Px_sym = sym(zeros(n,1));
    Py_sym = sym(zeros(n,1));

    for j = 1:n
        alpha = phi(j);
        arg = r*sin(theta_sym - alpha);
        sq = l^2 - arg^2;
        sq = piecewise(sq<0,0,sq);
        xp = r*cos(theta_sym - alpha) + sqrt(sq);
        Px_sym(j) = xp*cos(alpha);
        Py_sym(j) = xp*sin(alpha);
    end

    %% High Resolution θ sampling
    N = 10000;
    theta_vals = linspace(0, 2*pi, N);

    %% Convert theta → time
    t_vals = theta_vals / base_speed;

    %% Evaluate symbolic expressions
    Px_vals  = double(subs(Px_sym(1), theta_sym, theta_vals));
    Py_vals  = double(subs(Py_sym(1), theta_sym, theta_vals));

    % --- Shift position so that mean = 0
    Px_vals_centered = Px_vals - mean(Px_vals);
    Py_vals_centered = Py_vals - mean(Py_vals);

    %% Derivatives for velocity and acceleration
    dPx_dtheta   = diff(Px_sym(1), theta_sym);
    dPy_dtheta   = diff(Py_sym(1), theta_sym);
    d2Px_dtheta2 = diff(dPx_dtheta, theta_sym);
    d2Py_dtheta2 = diff(dPy_dtheta, theta_sym);

    dPx_vals  = double(subs(dPx_dtheta, theta_sym, theta_vals));
    dPy_vals  = double(subs(dPy_dtheta, theta_sym, theta_vals));
    d2Px_vals = double(subs(d2Px_dtheta2, theta_sym, theta_vals));
    d2Py_vals = double(subs(d2Py_dtheta2, theta_sym, theta_vals));

    %% Convert to real values (use centered Px)
    % Since derivative of a constant shift = 0, velocity/acceleration remain correct
    v_radial = (dPx_vals*cos(phi(1)) + dPy_vals*sin(phi(1))) * base_speed;
    a_radial = (d2Px_vals*cos(phi(1)) + d2Py_vals*sin(phi(1))) * base_speed^2;

    %% ==========================================================
    %%  FIGURE 1 — Plots vs theta (centered)
    %% ==========================================================
    figure('Color','w','Position',[300 50 900 900]);

    % ------------------
    % 1) Displacement
    % ------------------
    subplot(3,1,1);
    plot(theta_vals, Px_vals_centered, 'k','LineWidth',1.8);
    grid on;
    xlabel('\theta (rad)','FontSize',12);
    ylabel('Position (m)','FontSize',12);
    title('Piston #1 Displacement vs \theta (centered at 0)','FontSize',14);
    xlim([0 2*pi]);
    xticks(0:pi/6:2*pi);
    set(gca,'MinorGridLineStyle','-','XMinorGrid','on','YMinorGrid','on');

    % ------------------
    % 2) Velocity
    % ------------------
    subplot(3,1,2);
    plot(theta_vals, v_radial, 'b','LineWidth',1.8);
    grid on;
    xlabel('\theta (rad)','FontSize',12);
    ylabel('Velocity (m/s)','FontSize',12);
    title('Piston #1 Radial Velocity vs \theta','FontSize',14);
    xlim([0 2*pi]);
    xticks(0:pi/6:2*pi);
    set(gca,'MinorGridLineStyle','-','XMinorGrid','on','YMinorGrid','on');

    % ------------------
    % 3) Acceleration
    % ------------------
    subplot(3,1,3);
    plot(theta_vals, a_radial, 'r','LineWidth',1.8);
    grid on;
    xlabel('\theta (rad)','FontSize',12);
    ylabel('Accel (m/s^2)','FontSize',12);
    title('Piston #1 Radial Acceleration vs \theta','FontSize',14);
    xlim([0 2*pi]);
    xticks(0:pi/6:2*pi);
    set(gca,'MinorGridLineStyle','-','XMinorGrid','on','YMinorGrid','on');


    %% ==========================================================
    %%  FIGURE 2 — Plots vs Time (centered)
    %% ==========================================================
    figure('Color','w','Position',[1300 50 900 900]);

    % ------------------
    % 1) Displacement vs time
    % ------------------
    subplot(3,1,1);
    plot(t_vals, Px_vals_centered, 'k','LineWidth',1.8);
    grid on;
    xlabel('Time (s)','FontSize',12);
    ylabel('Position (m)','FontSize',12);
    title('Piston #1 Displacement vs Time (centered at 0)','FontSize',14);
    set(gca,'MinorGridLineStyle','-','XMinorGrid','on','YMinorGrid','on');

    % ------------------
    % 2) Velocity vs time
    % ------------------
    subplot(3,1,2);
    plot(t_vals, v_radial, 'b','LineWidth',1.8);
    grid on;
    xlabel('Time (s)','FontSize',12);
    ylabel('Velocity (m/s)','FontSize',12);
    title('Piston #1 Radial Velocity vs Time','FontSize',14);
    set(gca,'MinorGridLineStyle','-','XMinorGrid','on','YMinorGrid','on');

    % ------------------
    % 3) Acceleration vs time
    % ------------------
    subplot(3,1,3);
    plot(t_vals, a_radial, 'r','LineWidth',1.8);
    grid on;
    xlabel('Time (s)','FontSize',12);
    ylabel('Accel (m/s^2)','FontSize',12);
    title('Piston #1 Radial Acceleration vs Time','FontSize',14);
    set(gca,'MinorGridLineStyle','-','XMinorGrid','on','YMinorGrid','on');

end

%% Parameters (same as your original)
function [n,r,l,phi,maxR,pent_size,piston_w,piston_h,link_width,base_speed] = engine_params()
    n=5; r=0.01; l=0.047; pent_size=0.008;
    piston_w=0.015; piston_h=0.029; link_width=2; base_speed=10;
    phi=linspace(0,2*pi,n+1); phi(end)=[];
    maxR=r+l+0.1;
end
