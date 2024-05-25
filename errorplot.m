function errorplot(t,x_errors,y_errors,x0)
figure(2)
% subplot(211)
% stairs(t,x_errors,'r','linewidth',1.5); axis([0 t(end) -5 1.5])
% title('Deviation from x trajectory')
% xlabel('time (seconds)')
% ylabel('x errors (m)')
% grid on
% subplot(212)
% stairs(t,y_errors,'r','linewidth',1.5); axis([0 t(end) -5 1.5])
% title('Deviation from y trajectory')
% xlabel('time (seconds)')
% ylabel('y errors (m)')
% grid on


figure(4)
stairs(t,x_errors,'r','linewidth',1.5); axis([0 t(end) -5 1.5])
hold on 
stairs(t,y_errors,'g','linewidth',1.5); axis([0 t(end) -5 1.5])
title('trajectory Deviation')
legend('x errors','y errors')
xlabel('time (seconds)')
ylabel('errors (m)')
grid on


end