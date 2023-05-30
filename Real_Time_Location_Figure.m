function Real_Time_Location_Figure(x,z,yaw,t_x,t_z,arrow_length,r,r_x,r_z)
%REAL_TIME_LOCATION_FIGURE Summary of this function goes here
%   Detailed explanation goes here
circle_center =[r_x,r_z];
circle_radius =r;
circle_theta = linspace(0,2*pi,100);
circle_x=circle_center(1)+circle_radius*cos(circle_theta);
circle_y=circle_center(2)+circle_radius*sin(circle_theta);

fig = findall(0,'Type','Figure');
if isempty(fig)
    fig = figure();
end

ax = axes('Parent',fig);
cla(ax); % Clears the current axes
hold(ax, 'on');

plot(ax,circle_x,circle_y,'b-');
xlabel('X')
ylabel('Z')

plot(ax,0,0,'.');
plot(ax,t_x,t_z,'.','Color','r','MarkerSize',20);

ax.DataAspectRatio=[1 1 1];
dy=arrow_length*cosd(yaw);
dx=arrow_length*sind(yaw);
quiver(x,z,dx,dy,'r','LineWidth',0.2,'MaxHeadSize',2);

plot(x,z,'Color','r');
set(gca,'XDir','reverse');
xlim(ax,[-3,3]);
ylim(ax,[-2,2]);

hold(ax, 'off');
drawnow;
end
