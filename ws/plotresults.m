function plotresults(soln)

x = soln.x;
t = soln.t;
X = soln.X;

N = 100;
dt = (t(end)-t(1))/N;

time = t(1):dt:t(end);

points = [];
velocity = [];
for zt = time
    
    [p,v] = x2path(zt,t,X,soln.constraints.params);
    points = [points;   p];
    velocity = [velocity;   v];

end

figure;
plot3(points(:,1),points(:,2),points(:,3),'linewidth',2);
grid on;

color = {'r','g','b'};
vcolor = {'-or','-og','-ob'};
figure; hold on;
for i = 1:soln.constraints.params.m
    subplot(1,2,1); hold on;
    plot(time',points(:,i),color{i},'linewidth',1);
    grid on;
    subplot(1,2,2); hold on;
    plot(time',velocity(:,i),vcolor{i},'linewidth',1);
    grid on;
end
subplot(1,2,1); legend('x','y','z');
subplot(1,2,2); legend('vx','vy','vz');

keyboard;

end


%%
function [path_point, path_velocity] = x2path(t,T,X,params)

if t < min(T) || t > max(T)
    error('t is not in the range of ti');
end

for i = 1:params.n
    if t >= T(i) && t<T(i+1)
        powers = 0:1:params.d(i);
        time = (t*[ones(1,params.ncoef)]).^powers;
        path_point = [time*X(i,:,1)',time*X(i,:,2)',time*X(i,:,3)']; 
        
        if(length(time)>1)
            path_velocity = [time(1:end-1)*poly_diff(X(i,:,1),1)', time(1:end-1)*poly_diff(X(i,:,2),1)', time(1:end-1)*poly_diff(X(i,:,3),1)'];
        else
            path_velocity = [0,0,0];    
        end
        %[time*X(:,end,1);time*X(:,end,2);time*X(:,end,3)]';
    elseif t == T(end)
        disp('THE END');
        powers = 0:1:params.d(i);
        time = (t*[ones(1,params.ncoef)]).^powers;
        path_point = [time*X(params.n,:,1)',time*X(params.n,:,2)',time*X(params.n,:,3)']; 
        
        if(length(time)>1)
            path_velocity = [time(1:end-1)*poly_diff(X(i,:,1),1)', time(1:end-1)*poly_diff(X(i,:,2),1)', time(1:end-1)*poly_diff(X(i,:,3),1)'];
        else
            path_velocity = [0,0,0];    
        end
        %[time*X(:,end,1);time*X(:,end,2);time*X(:,end,3)]';main
    end
end


end












