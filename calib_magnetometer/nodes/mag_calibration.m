clc
clear

filename = ['2025-03-28-10-46-59.bag'];

bag = rosbag(filename);

tpc ='/alpha_rise/imu/mag';

bSel = select(bag,'Topic',tpc);
msgStructs = readMessages(bSel,'DataFormat','struct');

reft = bag.StartTime;

x = cellfun(@(m) double(m.MagneticField_.X),msgStructs);
y= cellfun(@(m) double(m.MagneticField_.Y),msgStructs);
z= cellfun(@(m) double(m.MagneticField_.Z),msgStructs);

t = bSel.MessageList.Time-reft;
figure
plot(t,x)
hold on
plot(t,y)
plot(t,z)

D = [x y z];

[A,b,expmfs] = magcal(D);

C = (D-b)*A;

figure
plot3(x(:),y(:),z(:),"LineStyle","none","Marker","X","MarkerSize",8)
hold on
grid(gca,"on")
plot3(C(:,1),C(:,2),C(:,3),"LineStyle","none","Marker","o", ...
      "MarkerSize",8,"MarkerFaceColor","r")
axis equal
xlabel("uT")
ylabel("uT")
zlabel("uT")
legend("Uncalibrated Samples","Calibrated Samples","Location","southoutside")
title("Uncalibrated vs Calibrated" + newline + "Magnetometer Measurements")
hold off