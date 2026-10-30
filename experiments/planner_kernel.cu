// Double-precision, no fast-math. One complete RK4 trajectory per CUDA thread.
__device__ void command(double t,const double* z,double* q,double* angle) {
    double t1=z[7],restart=t1+z[8],t2=z[9],finish=restart+t2,ramp=.5;
    if(t<5) *angle=0;
    else if(t<.2*t1) *angle=z[0]*(t-5)/(.2*t1-5);
    else if(t<.6*t1) *angle=z[0]+(z[1]-z[0])*(t-.2*t1)/(.4*t1);
    else if(t<t1-ramp) *angle=z[1]+(z[2]-z[1])*(t-.6*t1)/(.4*t1-ramp);
    else if(t<t1) *angle=z[2];
    else if(t<restart) *angle=z[2]+(z[3]-z[2])*(t-t1)/(restart-t1);
    else if(t<restart+ramp) *angle=z[3];
    else if(t<restart+.5*t2) *angle=z[3]+.5*(z[4]-z[3])*(t-restart-ramp)/(.5*t2-ramp);
    else if(t<finish-ramp) *angle=.5*(z[3]+z[4])+.5*(z[4]-z[3])*(t-restart-.5*t2)/(.5*t2-ramp);
    else *angle=z[4];
    if(t<t1-ramp) *q=z[5];
    else if(t<t1) *q=z[5]*(t1-t)/ramp;
    else if(t<restart) *q=0;
    else if(t<restart+ramp) *q=z[6]*(t-restart)/ramp;
    else if(t<finish-ramp) *q=z[6];
    else *q=z[6]*fmax(0.,(finish-t)/ramp);
}
__device__ void rhs(double t,const double* y,const double* z,const double* noise,
                    int noiseN,const double* p,double* dy) {
    double r=y[0],u=y[1],w=y[2],m=y[3],q,angle;
    command(t,z,&q,&angle);
    double V=sqrt(u*u+w*w),beta=V>1e-9?atan2(w,u):0.;
    double alpha=t<5?0.:beta+angle*.017453292519943295;
    double rho=p[2]*exp(-fmax(r-p[1],0.)/p[3]),K=.5*p[4]*p[5]*rho;
    int i=(int)floor(t);double f=t-i,norm=sqrt((1-f)*(1-f)+f*f);
    i=i<noiseN-1?i:noiseN-2;
    double ar=p[8]*rho/p[10]*((1-f)*noise[i]+f*noise[i+1])/norm;
    double aw=p[9]*rho/p[10]*((1-f)*noise[noiseN+i]+f*noise[noiseN+i+1])/norm;
    dy[0]=u;
    dy[1]=w*w/r+p[6]*q*cos(alpha)/m-p[0]/(r*r)-K*V*u/m+ar;
    dy[2]=-u*w/r+p[6]*q*sin(alpha)/m-K*V*w/m+aw;
    dy[3]=-q;
}
extern "C" __global__ void trajectories(const double* parameters,int count,const double* noise,
    int noiseN,const double* p,double step,double* values,double* terminal) {
    int row=blockIdx.x*blockDim.x+threadIdx.x;if(row>=count)return;
    const double* z=parameters+10*row;
    double used=z[5]*(z[7]-.25)+z[6]*(z[9]-.5);
    if(used>p[7]-p[11]) {values[row]=1000+pow((used-(p[7]-p[11]))/1000,2);return;}
    double finish=z[7]+z[8]+z[9],y[4]={p[1]+1,0,0,p[7]};
    int n=(int)ceil(finish/step);double dt=finish/n,impact=0;
    for(int k=0;k<n;++k) {
        double t=k*dt,k1[4],k2[4],k3[4],k4[4],temp[4];
        rhs(t,y,z,noise,noiseN,p,k1);
        for(int j=0;j<4;++j)temp[j]=y[j]+dt*k1[j]/2;
        rhs(t+dt/2,temp,z,noise,noiseN,p,k2);
        for(int j=0;j<4;++j)temp[j]=y[j]+dt*k2[j]/2;
        rhs(t+dt/2,temp,z,noise,noiseN,p,k3);
        for(int j=0;j<4;++j)temp[j]=y[j]+dt*k3[j];
        rhs(t+dt,temp,z,noise,noiseN,p,k4);
        for(int j=0;j<4;++j)y[j]+=dt*(k1[j]+2*k2[j]+2*k3[j]+k4[j])/6;
        if(y[0]<p[1]) {impact=100+finish-(t+dt);break;}
    }
    double vc=sqrt(p[0]/(p[1]+600000));
    double value=pow((y[0]-p[1]-600000)/10000,2)+pow(y[1]/1000,2)+pow((y[2]-vc)/1000,2)+impact;
    values[row]=isfinite(value)?value:1e12;
    for(int j=0;j<4;++j)terminal[4*row+j]=y[j];
}
