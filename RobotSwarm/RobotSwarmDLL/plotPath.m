function plotPath(a)

hold off;

%vl, vr in cm/s

vl = 6.5;
vr = 6.5;

d = 5.5; %cm

vl = vl + a;
vr = vr - a;

v = (vl + vr)/2;
w = (vl - vr)/d;


for t=0:0.1:100
   x = v/w*cos(w*t) - v/w;
   y = v/w*sin(w*t);
   plot(x,y,'-r');
   hold on;
end


vl = vl + a*0.5;
vr = vr - a*0.5;

v = (vl + vr)/2;
w = (vl - vr)/d;

for t=0:0.1:100
   x = v/w*cos(w*t) - v/w;
   y = v/w*sin(w*t);
   plot(x,y,'-b');
   hold on;
end


vl = vl + a*0.25;
vr = vr - a*0.25;

v = (vl + vr)/2;
w = (vl - vr)/d;

for t=0:0.1:100
   x = v/w*cos(w*t) - v/w;
   y = v/w*sin(w*t);
   plot(x,y,'-g');
   hold on;
end


% v = (vl + vr)/2;
% w = (vl - vr)/d;
% 
% for t=0:0.1:100
%    x = v/w*cos(w*t) - v/w;
%    y = v/w*sin(w*t);
%    plot(x,y);
%    hold on;
% end

end