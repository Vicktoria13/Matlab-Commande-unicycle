%% Suivi de trajectoire


function [X,Y] = generate_trajectoire(nb_points)
    % y = x
    a = 0.5
    b = 0.5

    theta =  linspace(0,5*pi,nb_points)/nb_points;

    X = linspace(0,200,nb_points)/nb_points;
    Y=0.3*X.^3;

    p1 = [3,5];
    p2 = [1,4];
    
    %radius of first point to second
    r = norm(p1-p2);
    %angle between two point wrt the y-axis
    theta_offset = tan((p1(2)- p2(2))/(p1(1)-p2(1)));
    
    rez = nb_points; % number of points 
    rev = 2; % number of revolutions
    
    t = linspace(0,r,rez); %radius as spiral decreases
    theta = linspace(0,2*pi*rev,rez) + theta_offset; %angle as spiral decreases
    X = cos(theta).*t+p2(1); 
    Y = sin(theta).*t+p2(2);

end


