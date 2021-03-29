% Convert the solution from spherical space to Cartesian coordinates

function position = SphericalToCart(sol,model)

    % Start location
    xs = model.start(1);
    ys = model.start(2);
    zs = model.start(3);
    
    % Solution in Sperical space
    r = sol.r;
    psi = sol.psi;
    phi = sol.phi;
    
    % First Cartesian coordinate
    x(1) = xs + r(1)*cos(psi(1))*sin(phi(1));
    
    % Check limits
    if x(1) > model.xmax
        x(1) = model.xmax;
    end
    if x(1) < model.xmin
        x(1) = model.xmin;
    end 
    
    y(1) = ys + r(1)*cos(psi(1))*cos(phi(1));
    if y(1) > model.ymax
        y(1) = model.ymax;
    end
    if y(1) < model.ymin
        y(1) = model.ymin;
    end
    
    z(1) = zs + r(1)*sin(psi(1));
    if z(1) > model.zmax
        z(1) = model.zmax;
    end
    if z(1) < model.zmin
        z(1) = model.zmin;
    end 
    
    % Next Cartesian coordinates
    for i = 2:model.n
        x(i) = x(i-1) + r(i)*cos(psi(i))*sin(phi(i));
        if x(i) > model.xmax
            x(i) = model.xmax;
        end
        if x(i) < model.xmin
            x(i) = model.xmin;
        end 

        y(i) = y(i-1) + r(i)*cos(psi(i))*cos(phi(i));
        if y(i) > model.ymax
            y(i) = model.ymax;
        end
        if y(i) < model.ymin
            y(i) = model.ymin;
        end

       % z(i) = z(i-1) + r(i)*cos(psi(i));
        z(i) = z(i-1) + r(i)*sin(psi(i));
        if z(i) > model.zmax
            z(i) = model.zmax;
        end
        if z(i) < model.zmin
            z(i) = model.zmin;
        end 
    end
    
    position.x = x;
    position.y = y;
    position.z = z;
end