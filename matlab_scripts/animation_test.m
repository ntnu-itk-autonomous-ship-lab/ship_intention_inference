x = 1:0.1:2;
y = x.^2;
y_2 = x.^0.5;

h = animatedline;
h2 = animatedline;

for i=1:length(x)
    addpoints(h,x(i),y(i));
end
drawnow;