fname = 'testTrack.json';
fid = fopen(fname);
raw = fread(fid,inf);
str = char(raw');
fclose(fid);
testTrack = jsondecode(str);

cones_blue(:,1) = testTrack.X_o;
cones_blue(:,2) = testTrack.Y_o;
cones_yellow(:,1) = testTrack.X_i;
cones_yellow(:,2) = testTrack.Y_i;
center_line(:,1) = testTrack.X;
center_line(:,2) = testTrack.Y;