clear all;
clc;

% x = -2:.2:2;
% y = -2:.25:2;
% z = -2:.16:2;
% 
% F = [x, y,z];
% figure
% [x,y,z] = meshgrid(x,y,z);
% mesh(F)


[x,y,z] = sphere; 
figure
patch(surf2patch(x,y,z,z)); 
shading faceted; 
view(3)