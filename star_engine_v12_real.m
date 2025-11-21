function star_engine_v13_real()
    close all; clc;

    %% Load geometric parameters
    [n, r, l, phi, maxR, pent_size, piston_w, piston_h, link_width, base_speed] = engine_params();
    visual_scale = 2;

    %% Symbolic variables
    syms theta_sym
    Px_sym = sym(zeros(n,1));
    Py_sym = sym(zeros(n,1));
    for j=1:n
        alpha = phi(j);
        arg = r*sin(theta_sym - alpha);
        sq = l^2 - arg^2;
        sq = piecewise(sq<0,0,sq);
        xp = r*cos(theta_sym - alpha) + sqrt(sq);
        Px_sym(j) = xp*cos(alpha);
        Py_sym(j) = xp*sin(alpha);
    end

    % Radial velocity symbolic - คำนวณ derivative
    dPx_dtheta = diff(Px_sym(1), theta_sym);
    dPy_dtheta = diff(Py_sym(1), theta_sym);
    
    % Radial acceleration symbolic - คำนวณ second derivative
    d2Px_dtheta2 = diff(dPx_dtheta, theta_sym);
    d2Py_dtheta2 = diff(dPy_dtheta, theta_sym);
    
    %% Compute piston stroke over full rotation
    theta_vals = linspace(0,2*pi,1000);
    Px_vals = double(subs(Px_sym(1),theta_sym,theta_vals));
    piston_stroke_real = max(Px_vals) - min(Px_vals);

    %% Main figure for animation
    fig = figure('Name','Star Engine Real Values','Color','w','Position',[100 100 1000 700]);
    axEng = axes('Parent',fig,'Position',[0.05 0.3 0.65 0.65]);
    hold(axEng,'on'); axis(axEng,'equal'); grid(axEng,'on');
    axis(axEng,[-maxR*visual_scale maxR*visual_scale -maxR*visual_scale maxR*visual_scale]);
    title(axEng,'Star Engine Animation (Real Values)','FontSize',14);

    % Crank pin
    h_crank_pin = plot(axEng,0,0,'ro','MarkerFaceColor','r','MarkerSize',6);

    % Pentagon
    pent_coords = pentagon_coords(pent_size);
    hPent = patch(axEng, pent_coords(1,:)*visual_scale, pent_coords(2,:)*visual_scale, [1 0.5 0],'FaceAlpha',0.6);
    pent_center_path = plot(axEng, NaN, NaN,'r--','LineWidth',1.2);

    % Pistons & links
    h_pistons = gobjects(n,1); h_links = gobjects(n,1); h_trails = gobjects(n,1);
    trajX = cell(n,1); trajY = cell(n,1); pistonCenters = gobjects(n,1);
    pistonLabels = gobjects(n,1);
    plotTrailFlag = true;

    % Central triangle
    triangle_size = 0.005;
    triangle_x = [0, -triangle_size*0.866, triangle_size*0.866, 0];
    triangle_y = [triangle_size, -triangle_size*0.5, -triangle_size*0.5, triangle_size];
    h_center = patch(axEng, triangle_x*visual_scale, triangle_y*visual_scale,'k','EdgeColor','k','FaceColor','none');

    % Link center line solid
    h_linkCenter = plot(axEng,[0 0],[0 0],'k-','LineWidth',1.5); 

    for j=1:n
        h_pistons(j) = patch(axEng,[0 0 0 0],[0 0 0 0],[0.2 0.5 1],'EdgeColor','k','FaceColor',[0.2 0.5 1]);
        h_links(j) = plot(axEng,[0 0],[0 0],'m','LineWidth',link_width);
        h_trails(j) = plot(axEng, NaN, NaN,'g--','LineWidth',1.2);
        trajX{j} = []; trajY{j} = [];
        pistonCenters(j) = plot(axEng,0,0,'go','MarkerFaceColor','g','MarkerSize',4);
        pistonLabels(j) = text(axEng,0,0,sprintf('%d',j),'FontSize',12,'FontWeight','bold',...
                               'Color','w','HorizontalAlignment','center','VerticalAlignment','middle');
    end

    hStrokeText = text(axEng, -maxR*visual_scale+0.01, maxR*visual_scale-0.02,...
                       sprintf('Piston stroke: %.3f m', piston_stroke_real),'FontSize',12,'FontWeight','bold');

    % Slider for crank speed
    uicontrol('Style','text','Parent',fig,'Units','normalized','Position',[0.75 0.85 0.15 0.03],'String','Crank Speed','FontSize',10);
    hSlider = uicontrol('Style','slider','Parent',fig,'Units','normalized','Position',[0.75 0.8 0.15 0.03],...
                        'Min',0,'Max',50,'Value',base_speed);
    hSliderText = uicontrol('Style','text','Parent',fig,'Units','normalized','Position',[0.75 0.77 0.15 0.03],...
                            'String',sprintf('%.1f rad/s',base_speed));

    %% Graph figure
    figGraph = figure('Name','Engine Graphs','Color','w','Position',[1200 200 600 1100]);
    axVel = subplot(4,1,1); hold(axVel,'on'); grid(axVel,'on');
    title(axVel,'Piston 1 Radial Velocity (real values)');
    xlabel(axVel,'Time [s]'); ylabel(axVel,'v_{radial} [m/s]');
    ylim(axVel,[-0.25 0.25]);
    hVelLine = plot(axVel,0,0,'b','LineWidth',1.5);
    hVelText = text(axVel,0.02,0.95,'','Units','normalized','FontSize',10,'VerticalAlignment','top');

    axVelAll = subplot(4,1,2); hold(axVelAll,'on'); grid(axVelAll,'on');
    title(axVelAll,'All Pistons Radial Velocity');
    xlabel(axVelAll,'Time [s]'); ylabel(axVelAll,'v_{radial} [m/s]');
    ylim(axVelAll,[-0.25 0.25]);
    colors = ['b','r','g','m','c'];
    hVelLineAll = gobjects(n,1);
    for j=1:n
        hVelLineAll(j) = plot(axVelAll,0,0,colors(j),'LineWidth',1.5,'DisplayName',sprintf('Piston %d',j));
    end
    legend(axVelAll,'Location','eastoutside');

    axOmega = subplot(4,1,3); hold(axOmega,'on'); grid(axOmega,'on');
    title(axOmega,'Crank Angular Velocity'); xlabel(axOmega,'Time [s]'); ylabel(axOmega,'Omega [rad/s]');
    hOmegaLine = plot(axOmega,0,0,'r','LineWidth',1.5);

    axAcc = subplot(4,1,4); hold(axAcc,'on'); grid(axAcc,'on');
    title(axAcc,'Piston 1 Acceleration (m/s^2)');
    xlabel(axAcc,'Time [s]'); ylabel(axAcc,'Acceleration');
    hAccLine = plot(axAcc,0,0,'k','LineWidth',1.5);
    hAccText = text(axAcc,0.02,0.95,'','Units','normalized','FontSize',10,'VerticalAlignment','top');

    %% Initial values
    theta_current = 0; dt=0.01; t=0;
    timeData=[]; velData=[]; omegaData=[]; accData=[];
    velDataAll = cell(n,1);
    for j=1:n
        velDataAll{j} = [];
    end
    v_max = -inf; v_min = inf;
    a_max = -inf; a_min = inf;

    %% Main loop
    while ishandle(fig) && ishandle(figGraph)
        t = t + dt;
        speed = get(hSlider,'Value');
        set(hSliderText,'String',sprintf('%.1f rad/s',speed));
        theta_current = theta_current + speed*dt;
        if theta_current > 2*pi
            theta_current = theta_current - 2*pi;
            plotTrailFlag = false;
        end

        % Evaluate symbolic positions
        Px_current = double(subs(Px_sym,theta_sym,theta_current));
        Py_current = double(subs(Py_sym,theta_sym,theta_current));
        Cx = r*cos(theta_current);
        Cy = r*sin(theta_current);

        % Update mechanism drawing
        set(h_crank_pin,'XData',Cx*visual_scale,'YData',Cy*visual_scale);
        set(hPent,'XData',(pent_coords(1,:)+Cx)*visual_scale,'YData',(pent_coords(2,:)+Cy)*visual_scale);
        if plotTrailFlag
            set(pent_center_path,'XData',[get(pent_center_path,'XData') Cx*visual_scale], ...
                                 'YData',[get(pent_center_path,'YData') Cy*visual_scale]);
        end

        for j=1:n
            R=[cos(phi(j)) -sin(phi(j)); sin(phi(j)) cos(phi(j))];
            corners=[-piston_w/2 -piston_h/2; piston_w/2 -piston_h/2; piston_w/2 piston_h/2; -piston_w/2 piston_h/2]';
            rotated=R*corners;
            set(h_pistons(j),'XData',rotated(1,:)*visual_scale + Px_current(j)*visual_scale,...
                             'YData',rotated(2,:)*visual_scale + Py_current(j)*visual_scale);
            set(h_links(j),'XData',[Cx + pent_coords(1,j) Px_current(j)]*visual_scale,...
                           'YData',[Cy + pent_coords(2,j) Py_current(j)]*visual_scale);

            if plotTrailFlag
                trajX{j}(end+1)=Px_current(j)*visual_scale; trajY{j}(end+1)=Py_current(j)*visual_scale;
                set(h_trails(j),'XData',trajX{j},'YData',trajY{j});
            end

            set(pistonCenters(j),'XData',Px_current(j)*visual_scale,'YData',Py_current(j)*visual_scale);
            set(pistonLabels(j),'Position',[Px_current(j)*visual_scale, Py_current(j)*visual_scale, 0]);
        end

        % Central link & triangle
        set(h_linkCenter,'XData',[0 Cx]*visual_scale,'YData',[0 Cy]*visual_scale);
        set(h_center,'XData',triangle_x*visual_scale,'YData',triangle_y*visual_scale);

        % คำนวณ Radial velocity - คูณกับ speed และ cos/sin ที่นี่
        dPx = double(subs(dPx_dtheta, theta_sym, theta_current));
        dPy = double(subs(dPy_dtheta, theta_sym, theta_current));
        v_piston_radial_plot = (dPx*cos(phi(1)) + dPy*sin(phi(1))) * speed;

        % คำนวณ Radial acceleration โดยใช้ symbolic second derivative
        d2Px = double(subs(d2Px_dtheta2, theta_sym, theta_current));
        d2Py = double(subs(d2Py_dtheta2, theta_sym, theta_current));
        acc_now = (d2Px*cos(phi(1)) + d2Py*sin(phi(1))) * speed^2;

        % คำนวณ velocity สำหรับทุก piston
        v_all_pistons = zeros(n,1);
        for j=1:n
            if j==1
                v_all_pistons(j) = v_piston_radial_plot;
            else
                % คำนวณ derivative สำหรับ piston อื่นๆ
                dPx_j = double(subs(diff(Px_sym(j), theta_sym), theta_sym, theta_current));
                dPy_j = double(subs(diff(Py_sym(j), theta_sym), theta_sym, theta_current));
                v_all_pistons(j) = (dPx_j*cos(phi(j)) + dPy_j*sin(phi(j))) * speed;
            end
        end

        % Update Max/Min velocity
        v_max = max(v_max, v_piston_radial_plot);
        v_min = min(v_min, v_piston_radial_plot);

        % Update history
        timeData=[timeData t];
        velData=[velData v_piston_radial_plot];
        omegaData=[omegaData speed];
        accData=[accData acc_now];
        for j=1:n
            velDataAll{j}(end+1) = v_all_pistons(j);
        end

        % Acceleration calculation (เอาออกเพราะคำนวณด้วย symbolic แล้ว)
        % if length(velData)>=2
        %     acc_now = (velData(end)-velData(end-1))/dt;
        % else
        %     acc_now = 0;
        % end
        % accData(end+1) = acc_now;

        % Update Max/Min acceleration
        a_max = max(a_max, acc_now);
        a_min = min(a_min, acc_now);

        % Update graphs
        set(hVelLine,'XData',timeData,'YData',velData);
        for j=1:n
            set(hVelLineAll(j),'XData',timeData,'YData',velDataAll{j});
        end
        set(hOmegaLine,'XData',timeData,'YData',omegaData);
        set(hAccLine,'XData',timeData,'YData',accData);

        % Update text annotations
        set(hVelText,'String',sprintf('Current: %.4f m/s\nMax: %.4f m/s\nMin: %.4f m/s', ...
            v_piston_radial_plot, v_max, v_min));
        set(hAccText,'String',sprintf('Current: %.4f m/s^2\nMax: %.4f m/s^2\nMin: %.4f m/s^2', ...
            acc_now, a_max, a_min));

        xlim(axVel,[max(0,t-10) t]);
        xlim(axVelAll,[max(0,t-10) t]);
        xlim(axOmega,[max(0,t-10) t]);
        xlim(axAcc,[max(0,t-10) t]);

        drawnow limitrate;
        pause(dt);
    end
end

%% Parameters
function [n,r,l,phi,maxR,pent_size,piston_w,piston_h,link_width,base_speed] = engine_params()
    n=5; r=0.01; l=0.047; pent_size=0.008;
    piston_w=0.015; piston_h=0.029; link_width=2; base_speed=10;
    phi=linspace(0,2*pi,n+1); phi(end)=[];
    maxR=r+l+0.1;
end

%% Pentagon vertices
function coords=pentagon_coords(size)
    ang=linspace(0,2*pi,6); coords=[cos(ang); sin(ang)]*size;
end