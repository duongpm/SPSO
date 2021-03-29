%{
 This function will plot:
- model with a terrain map and obstacles
- solutions with different views
%}

function PlotSolution(sol,model,smooth)

    %% Plot 3D view
    figure(1)
    PlotModel(model)
    
    x=sol.x;
    y=sol.y;
    z=sol.z;
    
    % Start location
    xs=model.start(1);
    ys=model.start(2);
    zs=model.start(3);
    
    % Final location
    xf=model.end(1);
    yf=model.end(2);
    zf=model.end(3);
    
    x_all = [xs x xf];
    y_all = [ys y yf];
    z_all = [zs z zf];
    
    N = size(x_all,2); % real path length
    
   % Path height is relative to the ground height
    for i = 1:N
        z_map = model.H(round(y_all(i)),round(x_all(i)));
        z_all(i) = z_all(i) + z_map;
    end
    
    % given data in a point matrix, xyz, which is 3 x number of points
    xyz = [x_all;y_all;z_all];
    [ndim,npts]=size(xyz);
    xyzp=zeros(size(xyz));
    for k=1:ndim
       xyzp(k,:)=ppval(csaps(1:npts,xyz(k,:),smooth),1:npts);
    end
    plot3(xyzp(1,:),xyzp(2,:),xyzp(3,:),'k','LineWidth',2);

    % plot start point
    plot3(x_all(1),y_all(1),z_all(1),'ks','MarkerSize',7,'MarkerFaceColor','k');
    % plot target point
    plot3(x_all(N),y_all(N),z_all(N),'ko','MarkerSize',7,'MarkerFaceColor','k');
    hold off;
    
    %% Plot top view
    figure(3)
    mesh(model.X,model.Y,model.H); % Plot the data
    colormap summer;                    % Default color map.
    set(gca, 'Position', [0 0 1 1]); % Fill the figure window.
    axis equal vis3d on;            % Set aspect ratio and turn off axis.
    shading interp;                  % Interpolate color across faces.
    material dull;                   % Mountains aren't shiny.
    camlight left;                   % Add a light over to the left somewhere.
    lighting gouraud;                % Use decent lighting.
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    hold on
    
    % Threats as cylinders
    threats = model.threats;
    threat_num = size(threats,1);
    
    for i = 1:threat_num
        threat = threats(i,:);
        threat_x = threat(1);
        threat_y = threat(2);
        threat_z = max(max(model.H))+1;  % choose z to be the highest peak
        threat_radius = threat(4);

        for j=1:3 
        % Define circle parameters:
        % Make an array for all the angles:
        theta = linspace(0, 2 * pi, 2000);
        % Create the x and y locations at each angle:
        x = threat_radius * cos(theta) + threat_x;
        y = threat_radius * sin(theta) + threat_y;
        % Need to make a z value for every (x,y) pair:
        z = zeros(1, numel(x)) + threat_z;
        % Do the plot:
        % First plot the center:
        plot3(threat_x, threat_y, threat_z, 'o', 'color', '#752e29', 'MarkerSize', 3, 'MarkerFaceColor','#752e29');
        % Next plot the circle:
        plot3(x, y, z, '-', 'color', '#752e29', 'LineWidth', 1);
        
        % Repeat for a smaller radius
        threat_radius = threat_radius - 20;
        end
    end

    % plot path
    plot3(xyzp(1,:),xyzp(2,:),xyzp(3,:),'k','LineWidth',2);

    % plot start point
    plot3(x_all(1),y_all(1),z_all(1),'ks','MarkerSize',7,'MarkerFaceColor','k');

    % plot target point
    plot3(x_all(N),y_all(N),z_all(N),'ko','MarkerSize',7,'MarkerFaceColor','k');
    
    % Set top view
    view(0,90)
    hold off;
    
    
    %% Plot side view
    figure(5)
    mesh(model.X,model.Y,model.H); % Plot the data
    colormap summer;                    % Default color map.
    set(gca, 'Position', [0 0 1 1]); % Fill the figure window.
    axis equal vis3d on;            % Set aspect ratio and turn off axis.
    shading interp;                  % Interpolate color across faces.
    material dull;                   % Mountains aren't shiny.
    camlight left;                   % Add a light over to the left somewhere.
    lighting gouraud;                % Use decent lighting.
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    hold on

    % plot path
    plot3(xyzp(1,:),xyzp(2,:),xyzp(3,:),'k','LineWidth',2);

    % plot start point
    plot3(x_all(1),y_all(1),z_all(1),'ks','MarkerSize',7,'MarkerFaceColor','k');

    % plot target point
    plot3(x_all(N),y_all(N),z_all(N),'ko','MarkerSize',7,'MarkerFaceColor','k');
    
    view(90,0);
    hold off;

end