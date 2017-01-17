x0 = 100;
y0 = 200;
varianceX = 25;
varianceY = 25;
X = linspace(1,4,4);
X = X';

Y = linspace(1,10,10);
Y=Y';
% x = [1;2;3]
% y = [1;2]
% size(X,1)
% size(X,2)
% size(Y,1)
% size(Y,2)
% size(x,1)
% size(x,2)
% size(y,1)
% size(y,2)
% Z = zeros(size(x,1),size(y,1));
% size(Z,1)
% size(Z,2)
% Z(1,2) = 10;
% X = [1;2;3];
% Y =[2;3];
% Z = zeros(2,3);
Z=zeros(size(Y,1),size(X,1));

% Z(1,1) = 10;
k = 1;
disp('Hello')
% z = linspace(1,40,40);
% plot(z);
% for k = 1:10
%    disp(z(k,1)); 
% end
%   for i = 1:3
%      for j = 1:9
%      Z(j,i) = z(k,1);
%      k = k +1;
%      end
%  end
T = .5.*(-9.8).*z.^2;
%T = z;
 Z(1,1) = T(1,3);
 surf(X,Y,Z);
% hold on;
%pause(10);
%