clear all;
clc;
%%set axis and figure
%myaxes = axes('xlim', [-2, 2],'ylim',[-2, 10],'zlim' ,[-1.5, 1.5]);
myaxes = axes('xlim', [-1000, 1000],'ylim',[-1000, 1000],'zlim' ,[-1000, 1000]);

view(3);
grid on;
%axis equal;
hold on;
xlabel('x')
ylabel('y')
zlabel('z')

%% generating the cylinder

[x,y,z] = cylinder([100, 100]);
h(1) = surface(x,y,z);
h(2) = surface(x,y,-z);
h(3) = surface(z,x,y);
h(4) = surface(-z,x,y);
% h(5) = surface(y,z,x,'FaceColor','magenta');
% h(6) = surface(y,-z,x,'FaceColor','yellow');

%% creating a group in matlab

combinedObject = hgtransform('Parent',myaxes);
set(h,'Parent', combinedObject);

drawnow

%% Defining the motion

longitude = 0:10;
latitude = [0 .1 .20 .40 .50 .80 1];
altitude = [0 .1 .20 .40 .50 .80 1];
bearing = 90;
%% step response

Kp = 25;
Kd = 15;
ki = 0;

num = [0, 1231.468, 1.6];
den = [2342269.5, 0, 87.808];
sys = tf(num, den);


error = 1;

c = pid(Kp,ki,Kd);
t = (0:0.1:20)';
tfinal  = t.^2;
T = feedback(sys*c,error);
Z = step(T, tfinal);
%% animation
for i = 1:length(Z)
    rotationOfCombinedObject = makehgtform('xrotate',(pi/180)*(bearing));
    set(combinedObject,'Matrix',rotationOfCombinedObject);

    TranslationOfCombinedObject = makehgtform('translate',[0, length(Z), 0]);
    set(combinedObject,'Matrix',TranslationOfCombinedObject)
    
    set(combinedObject,'Matrix',rotationOfCombinedObject*TranslationOfCombinedObject)
    
%TranslationOfCombinedObject = makehgtform('translate',[length(latitude), length(longitude), length(altitude)]);
%set(combinedObject,'Matrix',TranslationOfCombinedObject)

%pause(0.3)
end

figure(2)
plot(Z)
