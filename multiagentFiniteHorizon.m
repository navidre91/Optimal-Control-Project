clear; clc

Q=[1 0;0 3];
R=[1 0;0 1];
N=2;
T=30;
E=[1 0;0 1];

v1=[5;8];
v2=[70;-9];
v3=[30;-2];
v4=[10;9];



T=100;
dt=0.01;
t(1)=0;
index=1;

K = 2*E;

for k=T:-dt:0
    
    Kdot=2*N*Q-1/2*K(:,:,index)*R*K(:,:,index);
    K(:,:,index+1)=K(:,:,index)+Kdot*dt;
    tRec(index)=k;
    index=index+1;
end

t(1)=0;
index=1;

for k=0:dt:T
    
    F= 2*K(:,:,length(K)-index+1)^-1*Q;
    u1=-1/2*R^1*K(:,:,length(K)-index+1)*v1;
    u2=-1/2*R^1*K(:,:,length(K)-index+1)*v2;
    u3=-1/2*R^1*K(:,:,length(K)-index+1)*v3;
    u3=-1/2*R^1*K(:,:,length(K)-index+1)*v4;
    
    v1dot=u1+F*(v2+v3);
    v2dot=u2+F*(v1+v4);
    v3dot=u3+F*(v1+v2);
    v4dot=u3+F*(v2+v3);
   
    v1=v1+dt*v1dot;
    v2=v2+dt*v2dot;
    v3=v3+dt*v3dot;
    v4=v4+dt*v4dot;
    
    %data recording
    v1Rec(:,index)=v1;
    v2Rec(:,index)=v2;
    v3Rec(:,index)=v3;
    v4Rec(:,index)=v4;
    
    tRec(index)=k;
    index=index+1;
end

plot(v1Rec(2,:));hold on;plot(v2Rec(2,:));plot(v3Rec(2,:));plot(v4Rec(2,:));