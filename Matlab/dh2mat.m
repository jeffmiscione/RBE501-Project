function mat = dh2mat(theta, d, a, alpha, deg)
% set 'deg' to 1 to input angles as degrees or 0 for radians
if deg == 1;
    mat = [cosd(theta), -sind(theta)*cosd(alpha), sind(theta)*sind(alpha), a*cosd(theta);...
        sind(theta), cosd(theta)*cosd(alpha), -cosd(theta)*sind(alpha), a*sind(theta);...
        0, sind(alpha), cosd(alpha), d;...
        0, 0, 0, 1];
else
    mat = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);...
        sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);...
        0, sin(alpha), cos(alpha), d;...
        0, 0, 0, 1];
end