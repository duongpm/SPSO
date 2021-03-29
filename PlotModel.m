% Plot the terrain model and threats
function PlotModel(model)

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
    h=250; % Height
    
    for i = 1:threat_num
        threat = threats(i,:);
        threat_x = threat(1);
        threat_y = threat(2);
        threat_z = threat(3);
        threat_radius = threat(4);


        [xc,yc,zc]=cylinder(threat_radius); % create a unit cylinder
        % set the center and height 
        xc=xc+threat_x;  
        yc=yc+threat_y;
        zc=zc*h+threat_z;
        c = mesh(xc,yc,zc); % plot the cylinder 
        set(c,'edgecolor','none','facecolor','#FF0000','FaceAlpha',.3); % set color and transparency
    end

end