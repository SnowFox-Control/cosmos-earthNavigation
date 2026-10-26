function info=rocket_gpu_setup(requested)
info=struct('enabled',false,'name','','reason','Not requested');
if ~requested,return;end
try
    parallel.gpu.enableCUDAForwardCompatibility(true);
    d=gpuDevice;
    probe=gather(gpuArray(2)+gpuArray(3));assert(probe==5);
    info.enabled=true;info.name=d.Name;info.reason='CUDA forward compatibility enabled';
catch exception
    info.reason=exception.message;
    warning('rocket:GPUUnavailable','GPU unavailable: %s; using CPU parallel evaluation.',exception.message);
end
end
