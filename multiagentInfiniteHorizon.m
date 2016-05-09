%Infinite horizon simulation

clear; clc

Q=[1 0;0 3]; %Cost function wieghting 
R=[1 0;0 1]; %Cost function weighting 
N=2; %Number of neighboring agents
E=[1 0;0 1]; %Cost function weighting

v1=[5;8];  %Initial velocity of first agent
v2=[70;-9]; %Initial velocity of second agent
v3=[30;-2]; %Initial velocity of third agent
v4=[10;9]; %Initial velocity of fourth agent

x1=[20;1]; %Initial position of first agent
x2=[2;12]; %Initial position of second agent
x3=[14;8]; %Initial position of third agent
x4=[2;1]; %Initial position of fourth agent


T=5; %Total simulation time
dt=0.1; % time step
t(1)=0;
index=1;

%SteaDY state Optimal control gain calculation
K = care([0 0;0 0],R/sqrt(2),2*N*Q);
lamda=-1/2*R*K;

%Simulation of system
for k=0:dt:T
    
    %Optimal control gain calculation
    %Acceleration calculation
    v1dot=lamda*(v1-(v2+v3)/2);
    v2dot=lamda*(v2-(v1+v4)/2);
    v3dot=lamda*(v3-(v1+v2)/2);
    v4dot=lamda*(v4-(v2+v3)/2);
    
    %Positon calculation
    x1=x1+v1*dt;
    x2=x2+v2*dt;
    x3=x3+v3*dt;
    x4=x4+v4*dt;
    
    %Velocity calculation
    v1=v1+dt*v1dot;
    v2=v2+dt*v2dot;
    v3=v3+dt*v3dot;
    v4=v4+dt*v4dot;
    
    %data recording
    
    x1Rec(:,index)=x1;
    x2Rec(:,index)=x2;
    x3Rec(:,index)=x3;
    x4Rec(:,index)=x4;
    
    v1Rec(:,index)=v1;
    v2Rec(:,index)=v2;
    v3Rec(:,index)=v3;
    v4Rec(:,index)=v4;
    
    tRec(index)=k;
    index=index+1;
end

%ploting data

figure; hold on; box on;
p1 = plot(tRec,v1Rec(1,:),'r--');
p2 = plot(tRec,v2Rec(1,:),'m-');
p3 = plot(tRec,v3Rec(1,:),'b-o');
p4 = plot(tRec,v4Rec(1,:),'g-.');
set(p1,'LineWidth',2);
set(p2,'LineWidth',2);
set(p3,'LineWidth',2);
set(p4,'LineWidth',2);
set(gca,'Fontsize',14,'Fontweight','Bold')
xlabel({'\boldmath $t [s]$'},'FontSize',16,'Interpreter','latex');
ylabel({'\boldmath $v_{x}$'},'FontSize',16,'Interpreter','latex');
legend([p1 p2 p3 p4],[{'\boldmath $v_{x1}$'} {'\boldmath $v_{x2}$'} {'\boldmath $v_{x3}$'} {'\boldmath $v_{x4}$'}],'Interpreter','latex')

figure; hold on; box on;
p1 = plot(tRec,v1Rec(2,:),'r--');
p2 = plot(tRec,v2Rec(2,:),'m-');
p3 = plot(tRec,v3Rec(2,:),'b-o');
p4 = plot(tRec,v4Rec(2,:),'g-.');
set(p1,'LineWidth',2);
set(p2,'LineWidth',2);
set(p3,'LineWidth',2);
set(p4,'LineWidth',2);
set(gca,'Fontsize',14,'Fontweight','Bold')
xlabel({'\boldmath $t [s]$'},'FontSize',16,'Interpreter','latex');
ylabel({'\boldmath $v_{y}$'},'FontSize',16,'Interpreter','latex');
legend([p1 p2 p3 p4],[{'\boldmath $v_{y1}$'} {'\boldmath $v_{y2}$'} {'\boldmath $v_{y3}$'} {'\boldmath $v_{y4}$'}],'Interpreter','latex')

figure; hold on; box on;
p1 = plot(x1Rec(1,:),x1Rec(2,:),'r--');
p2 = plot(x2Rec(1,:),x2Rec(2,:),'m-');
p3 = plot(x3Rec(1,:),x3Rec(2,:),'b-o');
p4 = plot(x4Rec(1,:),x4Rec(2,:),'g-.');
set(p1,'LineWidth',2);
set(p2,'LineWidth',2);
set(p3,'LineWidth',2);
set(p4,'LineWidth',2);
set(gca,'Fontsize',14,'Fontweight','Bold')
xlabel({'\boldmath $x$'},'FontSize',16,'Interpreter','latex');
ylabel({'\boldmath $y$'},'FontSize',16,'Interpreter','latex');
legend([p1 p2 p3 p4],[{'\boldmath $1$'} {'\boldmath $2$'} {'\boldmath $3$'} {'\boldmath $4$'}],'Interpreter','latex')