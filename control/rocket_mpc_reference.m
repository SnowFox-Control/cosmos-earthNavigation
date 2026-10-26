function ref=rocket_mpc_reference(result,endTime)
% Interpolate the saved PSO state trajectory and extend it by nominal coasting.
assert(isfield(result,'optimizer') && strcmpi(result.optimizer,'PSO'), ...
    'MPC requires a PSO reference; run run_rocket(''optimize'') first.');
assert(result.success,'The saved PSO trajectory does not meet insertion tolerances.');
ref.cutoff=result.out.t(end);ref.control=result.control;
ref.firstCutoff=result.z(8);ref.restart=result.z(8)+result.z(9);
time=result.out.t;state=result.out.y(:,1:5);p=result.p;
if endTime>ref.cutoff
    opts=odeset('RelTol',1e-10,'AbsTol',[1e-5 1e-12 1e-7 1e-7 1e-7],'MaxStep',1);
    [tc,yc]=ode45(@(t,x)rocket_dynamics(t,x,p,0,0),[ref.cutoff endTime],state(end,:).',opts);
    time=[time;tc(2:end)];state=[state;yc(2:end,:)];
end
interpolants=cell(1,5);
for j=1:5,interpolants{j}=griddedInterpolant(time,state(:,j),'pchip','nearest');end
angle=griddedInterpolant(result.control.t(:),result.control.delta(:),'linear','nearest');
flow=griddedInterpolant(result.control.t(:),result.control.q(:),'linear','nearest');
ref.state=@stateAt;ref.command=@commandAt;
    function x=stateAt(t)
        x=zeros(5,numel(t));for k=1:5,x(k,:)=interpolants{k}(t(:)).';end
    end
    function u=commandAt(t)
        u=[angle(t(:)).';flow(t(:)).'];u(:,t>=ref.cutoff)=0;
    end
end
