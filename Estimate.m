function [Range,Azimuth,Elevation] =Estimate(Loc_estimate_x,Loc_estimate_y,Loc_estimate_z)
Rad_pos=[0 0 0];
target_x=Loc_estimate_x;%Q_estimate(1);
target_y=Loc_estimate_y;%Q_estimate(2);
target_z=Loc_estimate_z;%Q_estimate(3);

%Compute Range
Range=sqrt((Rad_pos(1)-target_x)^2+(Rad_pos(2)-target_y)^2+(Rad_pos(3)-target_z)^2);
%Compute Azimuth
Azimuth = atan((target_y - Rad_pos(2))/(target_x - Rad_pos(1)))*57.3;

%Azimuth=atan((target_x-Rad_pos(1))/(target_y-Rad_pos(2)))*57.3;

%Compute Elevation
Elevation=atan((target_z-Rad_pos(3))/sqrt((target_y-Rad_pos(2)).^2+(target_x-Rad_pos(1))^.2))*57.3;
if Range<=120
    fprintf('Range is %f\n',Range);
    fprintf('Azimuth is %f\n',Azimuth);
    fprintf('Elevation is %f\n',Elevation);
else 
    fprintf('No more lock!');
end