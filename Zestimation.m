%% Read data
inputf = 'Test629_129_FPmany.txt'
inputf = '129_FPmany2.txt'
inputf = 'FP20_deg0.txt'

data1 = dlmread(inputf,' ');

strings = strsplit(inputf,{'_','.'});
longname = 0;
if size(strings)*[0;1] > 3
    longname = 1;
end
% output = strcat(strings(2),'.eps')
% output2 = char(output)

%% Get length
Dsize = size(data1);
Length = Dsize(1);

%% define Reference
RefPose = [0,400,400,0];% X Y Z theta
RefImage = [0,0,1,0]; %x y 1?scale theta

%% Reading the each val
ofset = 3;

time = zeros(Length-ofset,1);
X = zeros(Length-ofset,1);
Y = zeros(Length-ofset,1);  
Z = zeros(Length-ofset,1);
Theta = zeros(Length-ofset,1);
x = zeros(Length-ofset,1);
y = zeros(Length-ofset,1);
inv_s = zeros(Length-ofset,1);
theta = zeros(Length-ofset,1);
count = zeros(Length-ofset,1);

% times = 1;
start=data1(1+ofset,1); 
for j = 1 : Length-ofset
    i=j+ofset;
   count(j) = j;
   time(j) = (data1(i,1)-start)/1000;
   X(j) = data1(i,3)-RefPose(1);
   Y(j) = data1(i,5)-RefPose(2);
   Z(j) = data1(i,2)-RefPose(3);
   Theta(j) = (data1(i,4)-RefPose(4))*180/pi;
   x(j) = data1(i,6)-RefImage(1);
   y(j) = data1(i,7)-RefImage(2);
   inv_s(j) = data1(i,9);%-RefImage(3);
   theta(j) = (data1(i,8)-RefImage(4))*180/pi;
end


%% put figure
% figure(1);
% [hAx0,hLine01,hLine02] = plotyy([time,time,time],[x,y,inv_s],time,theta);
% xlabel(hAx0(1),'Time [s]');
% ylabel(hAx0(1),'Error [pix]');
% ylabel(hAx0(2),'Error [degree]');
% legend0=legend('$\xi_x$','$\xi_y$','$\kappa$','$\Theta$');
% set(legend0,'Interpreter','latex');
% grid on
% xlim(hAx0(1),[0 25]);
% xlim(hAx0(2),[0 25]);
% % saveas(1,'sift_i1.pdf') ;
% 
% f2 = figure(2);
% [hAx2,hLine1,hLine2] = plotyy([time,time,time],[X,Y,Z],time,Theta);
% xlabel(hAx2(1),'Time [s]');
% ylabel(hAx2(1),'Error [mm]');
% ylabel(hAx2(2),'Error [degree]');
% legend1=legend('X','Y','Z','$\Theta$');
% set(legend1,'Interpreter','latex');
% grid on
% xlim(hAx2(1),[0 25]);
% xlim(hAx2(2),[0 25]);
% % saveas(2,'sift1.pdf') ;
% 
% f3 = figure(3);
% A = plot3(X,Y,Z,X(1),Y(1),Z(1),'o',X(j),Y(j),Z(j),'x');
% grid;
% legend('Robot Trajectory','Start','Goal');
% xlabel('X [mm]');
% ylabel('Y [mm]');
% zlabel('Z [mm]');

%% Output
% % output 
% %fig2
% if longname
%     output = char(strcat(strings(1),strings(2),strings(3),'_3D'));
% else
%     output = char(strcat(strings(1),strings(2),'_3D'));
% end
% SaveFigPDF(f2,output);
% 
% %fig3
% if longname
%     output = char(strcat(strings(1),strings(2),strings(3),'_3Dtraj'));
% else
%     output = char(strcat(strings(1),strings(2),'_3Dtraj'));
% end
% SaveFigPDF(f3,output);

%% Estimation
Len = size(X,1);
Z01 = zeros(Len,2);
Z02 = zeros(Len,2);
Z03 = zeros(Len,2);
Z04 = zeros(Len,2);
ff = 0.998
E1 = rls_const(1,ff); % Set Estimator1 
E2 = rls_const(1,ff); % Set Estimator2
E3 = rls_const(1,ff); % Set Estimator3
E4 = rls_const(1,ff); % Set Estimator4

% E1.set_Theta0([-1;1/250]);
% E2.set_Theta0([0;1/250]);
% E3.set_Theta0([0;250]);
% E4.set_Theta0([-1;250]);

E1.reinitialize([-1;1/10],10^7*eye(2),ff);
E2.reinitialize([0;1/10],10^7*eye(2),ff);
E3.reinitialize([0;200],10^9*eye(2),ff);
E4.reinitialize([-1;200],10^9*eye(2),ff);

for n = 60 : Len-2
    Z01(n,:) = ( E1.estimate(inv_s(n),[-inv_s(n-1);Z(n-2)-Z(n-3)]) )';
    Z02(n,:) = ( E2.estimate(inv_s(n)-inv_s(n-1), [- inv_s(n-1)+ inv_s(n-2); Z(n-2)-Z(n-3)]) )';
    Z03(n,:) = ( E3.estimate(Z(n)- Z(n-1), [- Z(n-1)+Z(n-2);inv_s(n+2)-inv_s(n+1)]) )';
    Z04(n,:) = ( E4.estimate(Z(n), [- Z(n-1);inv_s(n+2)-inv_s(n+1)]) )';
end

figure;
 plot(1./Z01(:,2))
ylim([0 500])
xlabel('Time');ylabel('distance Z_0');
grid on;
 figure;
 plot(1./Z02(:,2))
 ylim([0 500])
xlabel('Time');ylabel('distance Z_0');
grid on;

 figure; 
 plot(Z03)
ylim([0 500])
 figure;
plot(Z04)
ylim([0 500])
 
%% Estimation 2
E1x = rls_const(1,0.98);
E2x = rls_const(1,0.98);

Theta0 = 0;
for n =  90 : Len
    U = cos(Theta0)* (X(n-1)-X(n-2))+ sin(Theta0)* (Y(n-1)-Y(n-2));
    Z01(n,:) = ( E1x.estimate(x(n),[-x(n-1);U]) )';
    Z02(n,:) = ( E2x.estimate(x(n), [- x(n) + x(n-2);U]) )';
end

figure;
 plot(Z01)
 figure
 plot(Z02)
% %  ylim([-0.005 0.005])
% ylim([0 500])
% xlabel('Time')
%  figure;
%  plot(1./Z02(:,2))
%  ylim([0 500])

%% 2 sample delay
plot(theta(3:Len),Theta(1:Len-2))
grid on
