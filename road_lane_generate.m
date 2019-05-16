    %Lidar data generator
function [ out ] = road_lane_generate( name, point_n )

    %create file, deleteting old containing, if exists
fullname = [name, '.txt'];
fid = fopen(fullname, 'w+' );

    %we shold have at least 3 points to build plane
if point_n < 3
    point_n = 3;
end

    %find random sensor accuracy in range [a, b] 
a = 0.01;
b = 0.5;
acc = a + (b-a).*rand(1, 'double');
    
    %write data to file
fprintf(fid, '%f\n%d\n', acc, point_n);

conf = randi([1 ceil(point_n/2)]);

    %create 3 base points for plane building
a = -50;
b = 50;
x1 = a + (b-a).*rand(1, 'double');
x2 = a + (b-a).*rand(1, 'double');
x3 = a + (b-a).*rand(1, 'double');
y1 = a + (b-a).*rand(1, 'double');
y2 = a + (b-a).*rand(1, 'double');
y3 = a + (b-a).*rand(1, 'double');
a = 0;
b = 2;
z1 = a + (b-a).*rand(1, 'double');
z2 = a + (b-a).*rand(1, 'double');
z3 = a + (b-a).*rand(1, 'double');

    %write data to file
fprintf(fid, '%.2f %.2f %.2f\n%.2f %.2f %.2f\n%.2f %.2f %.2f\n', x1, y1, z1, x2, y2, z2, x3, y3, z3);

x_base = [x1; x2; x3];
y_base = [y1; y2; y3];
z_base = [z1; z2; z3];
    
    %building 3-point cloud and getting plane coefficients
pCloud = pointCloud([x_base, y_base, z_base]);
model = pcfitplane(pCloud,1);
A = model.Parameters(1,1);
B = model.Parameters(1,2);
C = model.Parameters(1,3);
D = model.Parameters(1,4);

    %generating plane-inlining points
i = 4;
while(i <= (fix(point_n/2) + conf))%i/point_n <= conf
    a = -50;
    b = 50;
    x = a + (b-a).*rand(1, 'double');
    y = a + (b-a).*rand(1, 'double');
    a = 0.01;
    b = acc;
    r = rand;
    if r > 0.5
        z = -(A*x+B*y+D)/C + (a + (b-a).*rand(1, 'double'));
    else
        z = -(A*x+B*y+D)/C - (a + (b-a).*rand(1, 'double'));
    end
    
        %write data to file
    fprintf(fid, '%.2f %.2f %.2f\n', x, y, z);
    
    i = i+1;
end

    %generating random noise
while(i/point_n <= 1)
    a = -50;
    b = 50;
    x = a + (b-a).*rand(1, 'double');
    y = a + (b-a).*rand(1, 'double');
    z = a + (b-a).*rand(1, 'double');
    
        %write data to file
    fprintf(fid, '%.2f %.2f %.2f\n', x, y, z);
    
    i = i+1;
end

    %wrriting output
out = {A, B, C, D, fullname};

fclose all;
end