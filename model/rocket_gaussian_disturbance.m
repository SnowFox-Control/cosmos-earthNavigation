function field=rocket_gaussian_disturbance(p,endTime)
% Seeded finite-band Gaussian acceleration, scaled by local density.
% Standardized linear interpolation keeps pointwise unit Gaussian variance.
% Absolute times allow the same realization across burn/coast restarts.
settings=struct('enabled',false,'sigma0',[0 0],'sampleInterval',1, ...
    'seed',20261020,'referenceDensity',1.225);
if isfield(p,'noise')
    names=fieldnames(p.noise);
    for j=1:numel(names),settings.(names{j})=p.noise.(names{j});end
end
validateattributes(settings.sigma0,{'numeric'},{'vector','numel',2,'finite','real','nonnegative'});
validateattributes(settings.sampleInterval,{'numeric'},{'scalar','finite','real','positive'});
validateattributes(settings.referenceDensity,{'numeric'},{'scalar','finite','real','positive'});
validateattributes(settings.seed,{'numeric'},{'scalar','integer','>=',0,'<=',2^32-1});
settings.sigma0=reshape(settings.sigma0,1,2);
active=settings.enabled && any(settings.sigma0>0) && p.rho0>0;
field.settings=settings;
field.maxStep=Inf;
if active
    dt=settings.sampleInterval;
    knots=(0:max(1,ceil(endTime/dt)+1))*dt;
    stream=RandStream('mt19937ar','Seed',settings.seed);
    samples=randn(stream,2,numel(knots)).'; % Stable prefixes for varying horizons
    radial=griddedInterpolant(knots(:),samples(:,1),'linear','nearest');
    tangential=griddedInterpolant(knots(:),samples(:,2),'linear','nearest');
    field.maxStep=dt/2;
end
field.evaluate=@evaluate;
    function acceleration=evaluate(time,height)
        time=time(:);height=height(:);
        acceleration=zeros(numel(time),2);
        if ~active,return;end
        time=max(0,time);
        lambda=mod(time/dt,1);
        normalization=sqrt((1-lambda).^2+lambda.^2);
        gaussian=[radial(time),tangential(time)]./normalization;
        rho=p.rho0*exp(-max(height,0)/p.H);
        acceleration=gaussian.*settings.sigma0.*(rho/settings.referenceDensity);
    end
end
