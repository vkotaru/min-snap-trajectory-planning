function [soln] = pathFinder(QUAD)
% Minimum snap trajectory generation

soln = []; % generated trajectory;

%% ---------------------------
%   EXTRACT PARAMETERS
% ----------------------------

pathx0 = QUAD.x0;
pathxf = QUAD.xf;
if size(pathx0) ~= size(pathxf)
    error('Dimensions of x0 and xf do not match');
end

% Scaling the distances
% ---------------------
lScale = 1;%norm(pathx0-pathxf);
pathx0 = pathx0/lScale;
pathxf = pathxf/lScale;

m = length(pathx0); % x dimension

% Path parameters
% ---------------
n = QUAD.params.n; % Number of segments
d = QUAD.params.d; % degree of the polynomial
switch QUAD.options.degtype
    case 'same';
        if ~(length(d)==1)
            error('same: d is should be of dimension 1x1');
        end
        d = repmat(d,1,n);
    case 'multiple';
        if (length(d) == 1)
            d = repmat(d,1,n);
        elseif ~(length(d)~= n)
            error('multiple: size of d should be equal to n');
        end
    otherwise
        error('selected an invalid option for polynomial degree');
end
ncoef = max(d)+1; % number of ceofficients

% time scaling
% ------------
alpha_t = 1;
% time frame
nframe = n+1;
switch QUAD.options.timeframe
    case 'equal';
        t = 0:(1/n):1;
    case 'diff';
        t = 0.1*ones(1,n);
    otherwise
        error('selected an invalid option for time frame');
end

% continuity
% -----------
r = QUAD.params.r; % order of continuity to be mainted between the segments

%% ------------------------
%   CONSTRAINTS FORMULATION
% -------------------------

% parameters
constraints.params = QUAD.params;
constraints.params.m = m;
constraints.params.nframe = nframe;
constraints.params.ncoef = ncoef;
constraints.params.x0 = pathx0;
constraints.params.xf = pathxf;
constraints.params.d = d;

A = [-eye(n), zeros(n,n*ncoef*m)];
b = zeros(n,1);
Aeq = [];
beq = [];
lb = [];
ub = [];
nonlcon = @(x) setNonLinConstraints(x,constraints.params);

%% --------------------
%   INTIAL GUESS
% ---------------------
% x = [alphat, t];
% x = [x, [[c0 c1 ... cd]_i]_j ]
x0 = [t'];

for i = 1:m
    x0 = [x0;ones(ncoef*n,1)];
end

%% ---------------------
%   fmincon
% ----------------------
% opts = optimoptions('quadprog',...
%     'Algorithm','interior-point-convex','Display','iter');
options = optimoptions('fmincon','Display','iter','Algorithm','sqp');

% Objective function
fun = @(x) costFunc(x,constraints.params);

[x,fval,exitflag,output]  = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon,options);

% Optimum time interval
t = x(1:n)';
t = [0,cumsum(t)];

soln.x = x;
soln.t = t;
soln.X = reshape(x(nframe:end),n,ncoef,m);
soln.constraints = constraints;


% temporary
% keyboard;

end

function[fval] = costFunc(x,params)

fval = 0;

% alphat = x(1);
t = x(1:params.n)';
t = [0,cumsum(t)];
X = reshape(x(params.nframe:end),params.n,params.ncoef,params.m);

for i = 1:size(X,3) % looping over different dimensions
     
    for j = 1:(params.n) % looping over different path segments
        t_ini = t(j);
        t_fin = t(j+1);
        
        dx = poly_diff(X(j,:,i),params.r);
        int_dx_dt = poly_int(dx,t_ini,t_fin,'coeff');
        fval = fval+(sum(int_dx_dt))^2;
    end
    
end



end


function[C, Ceq] = setNonLinConstraints(x,params)

C = [];
Ceq = [];

% alphat = x(1);
t = x(1:params.n)';
t = [0,cumsum(t)];
X = reshape(x(params.nframe:end),params.n,params.ncoef,params.m);

for i = 1:size(X,3) % looping over different dimensions
    % constraint on start position
    x0 = pos(t(1),params.d(1),X(1,:,i));
    Ceq = [Ceq; ((x0)-params.x0(i))];
    % constraint on start velocity and higher derivatives
    for ii = 1:params.rr
        vx0 = dpos(t(1),params.d(1),X(1,:,i),ii);
        Ceq = [Ceq; vx0-0];
    end
    
    for j = 1:(params.n-1) % looping over different path segments
        
        % continuity on position; constraints
        tempx1 = pos(t(j+1),params.d(j),X(j,:,i));
        tempx2 = pos(t(j+1),params.d(j+1),X(j+1,:,i));       
        Ceq = [Ceq; (tempx1-tempx2)];
        % cotinuity constraints on velocity and higher derivatives
        for ii = 1:params.rr
            tempvx1 = dpos(t(j+1),params.d(j),X(j,:,i),ii);
            tempvx2 = dpos(t(j+1),params.d(j+1),X(j+1,:,i),ii);
            Ceq = [Ceq; (tempvx1-tempvx2)];
        end
        
    end
    % constraint on end position
    xf = pos(t(end),params.d(params.n),X(params.n,:,i));
    Ceq = [Ceq; ((xf)-params.xf(i))];
    % constraint on end velocity and higher derivatives
    for ii = 1:params.rr
        vxf = dpos(t(end),params.d(params.n),X(params.n,:,i),ii);
        Ceq = [Ceq; vxf-0];
    end
    
end

function[px] = pos(t,d,X)
    deg = 0:1:d;
    time = (t*ones(1,length(deg))).^deg;
    px = sum(time.*X);
end

function[vx] = dpos(t,d,X,ord)
    deg = 0:1:d;
    time = (t*ones(1,length(deg))).^deg;
%     px = sum(time.*X);
    
    if(length(deg)>1)
        vx = time(1:end-ord)*poly_diff(X,ord)';
    else
        vx = 0;    
    end
    
end

end



