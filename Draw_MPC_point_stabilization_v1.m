function Draw_MPC_point_stabilization_v1 (t,xx,xx1,u_cl,xs,N,rob_diam,new_xs_x,new_xs_y,new_xs_theta,map)


set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', 12)

line_width = 1.5;
fontsize_labels = 14;

%--------------------------------------------------------------------------
%-----------------------Simulate robots -----------------------------------
%--------------------------------------------------------------------------
x_r_1 = [];
y_r_1 = [];
obs_x = 18; % meters
obs_y = 20; % meters
obs_diam = 2; % meters


r = rob_diam/2;  % obstacle radius
ang=0:0.005:2*pi;
xp=r*cos(ang);
yp=r*sin(ang);

 % obstacle radius
 r1 = obs_diam/2;
xp_obs=r1*cos(ang);
yp_obs=r1*sin(ang);

figure(500)
% Animate the robot motion
%figure;%('Position',[200 200 1280 720]);
set(gcf,'PaperPositionMode','auto')
set(gcf, 'Color', 'w');
set(gcf,'Units','normalized','OuterPosition',[0 0 0.55 1]);

for k = 1:size(xx,2)
    h_t = 0.14; w_t=0.09; % triangle parameters

    x1 = xs(1); y1 = xs(2); th1 = xs(3);
    x1_tri = [ x1+h_t*cos(th1), x1+(w_t/2)*cos((pi/2)-th1), x1-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
    y1_tri = [ y1+h_t*sin(th1), y1-(w_t/2)*sin((pi/2)-th1), y1+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];
    fill(x1_tri, y1_tri, 'g'); % plot reference state
    hold on;
    x1 = xx(1,k,1); y1 = xx(2,k,1); th1 = xx(3,k,1);
    x_r_1 = [x_r_1 x1];
    y_r_1 = [y_r_1 y1];
    x1_tri = [ x1+h_t*cos(th1), x1+(w_t/2)*cos((pi/2)-th1), x1-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
    y1_tri = [ y1+h_t*sin(th1), y1-(w_t/2)*sin((pi/2)-th1), y1+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];
    show(map);      

    plot(x_r_1,y_r_1,'-r','linewidth',line_width);hold on % plot exhibited trajectory
    if k < size(xx,2) % plot prediction
        plot(xx1(1:N,1,k),xx1(1:N,2,k),'r--*')
    end

    fill(x1_tri, y1_tri, 'r'); % plot robot position
    plot(x1+xp,y1+yp,'--r'); % plot robot circle
    plot(new_xs_x,new_xs_y);
    plot(obs_x+xp_obs,obs_y+yp_obs,'-b'); % plot obstacle circle 


    hold off
    %figure(500)
    ylabel('$y$-position (m)','interpreter','latex','FontSize',fontsize_labels)
    xlabel('$x$-position (m)','interpreter','latex','FontSize',fontsize_labels)
    axis([-0.2 50 -0.2 50])
    pause(0.1)
    box on;

    %aviobj = addframe(aviobj,gcf);
    drawnow
    % for video generation
    F(k) = getframe(gcf); % to get the current frame
end
% close(gcf)
% viobj = close(aviobj)
% video = VideoWriter('exp.avi','Uncompressed AVI');
% 
% video = VideoWriter('exp.avi','Motion JPEG AVI');
% video.FrameRate = 5;  % (frames per second) this number depends on the sampling time and the number of frames you have
% open(video)
% writeVideo(video,F)
% close (video)

figure(1)
stairs(t,u_cl(:,1),'k','linewidth',1.5); axis([0 t(end) -0.35 3])
title('Velocity plot')
xlabel('time (seconds)')
ylabel('v (m/s)')
grid on
figure(2)
stairs(t,u_cl(:,2),'r','linewidth',1.5); axis([0 t(end) -0.85 1.5])
title('angular Velocity plot')
xlabel('time (seconds)')
ylabel('\omega (rad/s)')
grid on
figure(3)
stairs(t,u_cl(:,1).^2,'g','linewidth',1.5); axis([0 t(end) -0.85 5])
title('Kinetic energy plot')
xlabel('time (seconds)')
ylabel('KE (j)')
grid on


