    %main algorithm function,
    %requires 'Image process' and 'Computer vision' toolkits
function [] = road_lane(info)
    %input data processing
tA = str2double(string(info(:,1)));
tB = str2double(string(info(:,2)));
tC = str2double(string(info(:,3)));
tD = str2double(string(info(:,4)));
file_name = string(info(:,5));
    
    %read file with data
data = dlmread(file_name);

    %process file data
maxDistance = data(1,1);
x = data(3:end,1);
y = data(3:end,2);
z = data(3:end,3);

    %creating point cloud
cloud = [x y z];
ptCloud = pointCloud(cloud);
n = size(cloud,1);

    %plot the point cloud
subplot(1,2,1)
pcshow(ptCloud)
str = ['(', num2str(tA), ')*x + ', '(', num2str(tB), ')*y + ', '(', num2str(tC), ')*z + (', num2str(tD), ') = 0'];
str = sprintf('Generated plane: %s', str);
title({['The whole cloud: ', num2str(n), ' points']; str})
xlabel('x');
ylabel('y');
zlabel('z');

    %Find the plane embracing max of point field 
    %in position closest to {0,0,1} plane normal.
    %Considering threshold corridor around the plane
referenceVector = [0,0,1];
maxAngularDistance = 20;
[model,inlierIndices] = pcfitplane(ptCloud,maxDistance, referenceVector,maxAngularDistance);

    %set final coefficient as variables
A = model.Parameters(1,1);
B = model.Parameters(1,2);
C = model.Parameters(1,3);
D = model.Parameters(1,4);

    %Find all points realted to new plane and plot them
points_in_plane = select(ptCloud,inlierIndices);
subplot(1,2,2)
pcshow(points_in_plane)
str = ['(', num2str(A), ')*x + ', '(', num2str(B), ')*y + ', '(', num2str(C), ')*z + (', num2str(D), ') = 0'];
str = sprintf('Resulted plane: %s', str);
title({['Related to plane: ', num2str(points_in_plane.Count), ' points']; str})
xlabel('x');
ylabel('y');
zlabel('z');

hold on;
    %adding green meshgrid with correct plane
[a, b] = meshgrid(-40:40,-40:40);
c = -(A*a + B*b + D)/C;
surf(a,b,c,'FaceColor','texturemap', 'FaceAlpha',0.2, 'EdgeColor', 'green')

    %adding red meshgrid with calculated plane
[x, y] = meshgrid(-30:30,-30:30);
z = -(tA*x + tB*y + tD)/tC;
surf(x,y,z,'FaceColor','texturemap', 'FaceAlpha',0.2, 'EdgeColor', 'red')

hold off;

    %print coefficients on the command line
fprintf('%f %f %f %f\n', A, B, C, D);
fclose all;
end
