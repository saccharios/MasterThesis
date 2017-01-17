function plotData(filename,dataSet, numberOfRobots)

close all;
A = importdata(filename);

if(strcmp(dataSet,'kalman'))
    
    figure('name','x-, y-states and x-,y-measurement plot');
    plot(A(:,1),'-b') %plot x-measurement
    hold on;

%     plot(A(:,2),'-r') %plot y-measurement
%     hold on;
% 
%     plot(A(:,3),'-g') %plot x-state
%     hold on;
% 
%     plot(A(:,4),'-c') %plot y-state
% 
%     legend('x-measurement','y-measurement','x-state','y-state',...
%                   'Location','NorthEastOutside')
%     xlabel('Iterations');
%     ylabel('cm');
%     
%     figure('name','dx-, dy-states plot');
%     plot(A(:,5),'-b') %plot dx-state
%     hold on;
%     
%     plot(A(:,6),'-r') %plot dy-state
%     hold on;
%     
%     legend('dx-state','dy-state',...
%                   'Location','NorthEastOutside')
%     xlabel('Iterations');
%     ylabel('cm');

elseif(strcmp(dataSet,'kalmanAllRobots'))
    
    
   % for j=1:numberOfRobots:size(A,1)
       for i=1:numberOfRobots

            figure(i);
            

            plot(A(i:numberOfRobots:size(A,1),1),'-b') %plot x-measurement
            hold on;

%             plot(A(i:numberOfRobots:size(A,1),2),'-r') %plot y-measurement
%             hold on;
% 
%             plot(A(i:numberOfRobots:size(A,1),3),'-g') %plot x-state
%             hold on;
% 
%             plot(A(i:numberOfRobots:size(A,1),4),'-c') %plot y-state
%             hold on;

%             legend('x-measurement','y-measurement','x-state','y-state',...
%                           'Location','NorthEastOutside')
%             xlabel('Iterations');
%             ylabel('cm');

%             figure(i+numberOfRobots);
%            
% 
%             plot(A(i:numberOfRobots:size(A,1),5),'-b') %plot dx-state
%             hold on;
% 
%             plot(A(i:numberOfRobots:size(A,1),6),'-r') %plot dy-state
%             hold on;

            legend('P(1,1)','Location','NorthEastOutside')
            xlabel('Iterations');
            ylabel('cm^2');
        
    end
   
elseif(strcmp(dataSet,'gauss'))
    
     WorldToPixX = 1/300*640;
    WorldToPixY = 1/234*480;
    
    x=linspace(1,640,640);  
    y=linspace(1,480,480);
    %generates a row vector y of 1000 points linearly spaced between and including -3 and 3
    %y=x;          
    [X,Y]=meshgrid(x,y);
    
    %for i=1:size(A,1)
    
    meanX = 400;% A(i,1);
    meanY = 200; %A(i,2);
    
    meanX2 = 301;% A(i,1);
    meanY2 = 200; %A(i,2);
    
    varianceX = 1000;
    varianceX = varianceX*WorldToPixX;
    varianceY = 1000;
    varianceY = varianceY*WorldToPixY;
    tmpX = X - meanX;
    tmpY = Y - meanY;
    
    tmpX2 = X - meanX2;
    tmpY2 = Y - meanY2;
    
    x0 = 350;
    y0 = 200;
    
    z=100*exp(-(tmpX.^2/(2*varianceX) + tmpY.^2/(2*varianceY)));
     z2=100*exp(-(tmpX2.^2/(2*varianceX) + tmpY2.^2/(2*varianceY)));
%     z=exp(-((X - meanX).^2+(Y - meanY).^2)/2);
F = zeros(2,1);
F2 = zeros(2,1);
for i = x0-16:x0+16
    for j = y0-16:y0+16
        d_quadrat = (i-x0)* (i-x0) + (j-y0)*(j-y0);
        z(j,i)
        if(d_quadrat ~= 0)
            F(1,1) = F(1,1) + z(j,i)/d_quadrat * (i-x0)/sqrt(d_quadrat)
            F(2,1) = F(2,1) + z(j,i)/d_quadrat * (j-y0)/sqrt(d_quadrat)
            F2(1,1) = F2(1,1) + z2(j,i)/d_quadrat * (i-x0)/sqrt(d_quadrat)
            F2(2,1) = F2(2,1) + z2(j,i)/d_quadrat * (j-y0)/sqrt(d_quadrat)
        end
        
    end
end
disp('final F');
F = F + F2;
F = -F;
disp(F);
 
   % F(1,1) = F(1,1)/sqrt(F(1,1)*F(1,1) + F(2,1)*F(2,1));
    % F(2,1) = F(2,1)/sqrt(F(1,1)*F(1,1) + F(2,1)*F(2,1));
     F = 100*F;
     x_new = x0 + F(1,1);
     y_new = y0 + F(2,1);
     %C = 100*ones(480,640);
     %C(y0,x0,100);
     surf(x,y,z);
     shading interp
     hold on

     
     surf(x,y,z2);
     shading interp
     hold on
     
    
    
     plot3(x0,y0,100,'--rs','LineWidth',2,...
'MarkerEdgeColor','k',...
'MarkerFaceColor','g',...
'MarkerSize',10)
    hold on;
 
    plot3(x_new,y_new,100,'--rs','LineWidth',2,...
'MarkerEdgeColor','k',...
'MarkerFaceColor','r',...
'MarkerSize',10)
   



   % end
    %nice3d();
    
    xWorld = zeros(2,1);
    yWorld = zeros(2,1);
    xPix = zeros(2,1);
    yPix = zeros(2,1);

%      3.49694e-012
%  1.98165e-012
% Pot: 6.11131e-012
% Pot: 5.87426e-012
% Pot: 3.32882e-012
% Pot: 1.0266e-011
% Pot: 2.06236e-012
% Pot: 1.1687e-012
% Pot: 3.60421e-012
    
%     varianceX = varianceX*WorldToPixX
%     varianceY = varianceY*WorldToPixY
%     
%     for i = 1:size(xWorld,1)
%         
%         xPix(i,1) = xWorld(i,1)*WorldToPixX
%         yPix(i,1) = yWorld(i,1)*WorldToPixY
% 
%         100*exp(-((i-200)^2/(2*varianceX) + (i-300)^2/(2*varianceY)))
%     
%     end
%     
%         mean=[8 2] ;
% sigma=[4.1 0;0 2.8];
% x = mvnrnd(mean,sigma,100) ;
% [X Y]=meshgrid(x(:,1),x(:,2));
% Z = peaks(X,Y);
% surf(X,Y,Z); %or mesh(X,Y,Z)
   
elseif(strcmp(dataSet,'wallpotential'))
    
     %x=linspace(-600,600,100);  
    %generates a row vector y of 1000 points linearly spaced between and including -3 and 3
    % y=x;          
    %[X,Y]=meshgrid(x,y);
    
%     x=linspace(1,640,640);
%     y=linspace(1,480,480);
%     z = zeros(640,480);
    %generates a row vector y of 1000 points linearly spaced between and including -3 and 3
              
%     [X,Y]=meshgrid(x,y);
    
    for i=1:size(A,1)
        for j=1:size(A,2)
         z(i,j)=A(i,j);
         
        end
    end
    
        %surf(x',y',g);
        surf(z);
         shading interp
         hold on
end
        


end