function out = draw_cart_pendulum(state,time,sampling_time)

for i = 1:time
    clf
    annotation('textbox',[0.25 0.002 0.3 0.3],'String','Inverted Pendulum on a Cart by Parham Oveissi','FitBoxToText','on')
    annotation('textbox',[0.15 0.6 0.3 0.3],'String',['Initial Conditions: ' num2str(rad2deg(state(1,3))) ' deg ' ' & ' num2str(state(1,1)) ' m'],'FitBoxToText','on')
    annotation('textbox',[0.6 0.6 0.3 0.3],'String',['Simulation Time: ' num2str((i-1)*sampling_time) ' s'], 'FitBoxToText','on')
    width = 10;
    height = 5;
    xCenter = state(i,1);
    yCenter = 4;
    xLeft = xCenter - width/2;
    yBottom = yCenter - height/2;
    rectangle('Position', [xLeft, yBottom, width, height], 'FaceColor', 'r');
    
    xlim([-40, 40]);
    ylim([-10,30]);
%     grid on;
    axis equal
    
    
    hold on
    x = state(i,1) ;
    y = yCenter+2;
    alpha = state(i,3)-pi/2;
    L =15;
    x2=x+(L*cos(alpha));
    y2=y+(L*sin(alpha));
    plot([x x2],[y y2],'b' ,'LineWidth', 4)
    
    pos1 = [xCenter-4 yCenter-4 2 2];
    pos2 = [xCenter+2 yCenter-4 2 2];
    pos3 = [xCenter-1 yCenter+1.5 2 2];
    rectangle('Position', pos1, 'FaceColor', 'g','Curvature',[1 1]);
    rectangle('Position', pos2, 'FaceColor', 'g','Curvature',[1 1]);
    rectangle('Position', pos3, 'FaceColor', 'm','Curvature',[1 1]);
    line([-100 100],[0 0],'color','k')
    pause(0.001)
end
annotation('textbox',[0.33 0.5 0.3 0.3],'String',['Final States: ' num2str(rad2deg(state(time,3))) ' deg ' ' & ' num2str(state(time,1)) ' m'],'FitBoxToText','on')

end