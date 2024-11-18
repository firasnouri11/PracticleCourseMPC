% MATLAB program for Linear MPC: Single-input system (alternate code for LMPC_1.m with constraints defined using lb and ub)
clear all;
close all
% System parameters and simulation parameters
A = [1 1; 0 1];
B = [0; 1];
NT=100;N=5;n=2;m=1; 
Q=eye(n); QN=Q; R=0 *eye(m); K = dlqr(A,B,Q,R); % at first we don't need K if we don't have any noise
x_min=[-1;-5];x_max=[60;5]; umin=-1;umax=1;
x_min = repmat(x_min,1,N+1); x_max = repmat(x_max,1,N+1);

x0=[0;0]; 
x=zeros(n,NT+1); x(:,1)=x0;
Xk=zeros(n*(N+1),1); Xk(1:n,1)=x0;
u=zeros(m,NT);
Uk=zeros(m*N,1);
zk=[Xk;Uk];
x_ref = zeros(n,(N+NT+1)); x_ref(1,10:40) = 60; x_ref(1,65:85) = 45;
u_ref = zeros((N+NT),1);

% define random noise
w = -0.1 + 0.2 * rand(1,NT);

% constructing AX,BU,QX,RU,H
for i=1:N+1
    AX((i-1)*n+1:i*n,:)=A^(i-1);
end
for i=1:N+1
  for j=1:N
      if i>j
          BU((i-1)*n+1:i*n,(j-1)*m+1:j*m)=A^(i-j-1)*B;
      else
          BU((i-1)*n+1:i*n,(j-1)*m+1:j*m)=zeros(n,m);
      end    
  end
end
QX=Q;RU=R;
for i=1:N-1
  QX=blkdiag(QX,Q); RU=blkdiag(RU,R);
end
QX=blkdiag(QX,QN);
H=blkdiag(QX,RU);

lb=[reshape(x_min,1,[]) umin*ones(1,N*m)]; % set lower bound for state x and input u
ub=[reshape(x_max,1,[]) umax*ones(1,N*m)];
x_predicted = zeros(n,NT+1);
u_predicted = zeros(m,NT);
% simulating system with MPC
for k=1:NT
   % define current state x
   xk=x(:,k);
   % define reference trajectory z_ref_iter = [x_ref;u_ref] in current
   % iteration
   z_ref_iter = [reshape(x_ref(:,k:(N+k)),[],1); u_ref(k:(N+k-1))];
   % define cost function as a reference tracking problem (input is not weighted) 
   fun = @(z)(z_ref_iter - z)'*H*(z_ref_iter -z);
   
   F=[];g=[];Feq=[eye((N+1)*n) -BU];geq=AX*xk;
   z=fmincon(fun,zk,F,g,Feq,geq,lb,ub);
   
   x_predicted(:,k) = z(1:n,1);
   u_predicted(:,k) = z((N+1)*n+1:(N+1)*n+m,1);
   u(:,k) = u_predicted(:,k) + K * (-xk + x_predicted(:,k)); % optimal input with stabilizing feedback control law K
   u(:,k) = max(min(u(:,k),umax), umin); % check plausibility
   x(:,k+1)= A * x(:,k) + B * u(:,k) + [0;1] * w(:,k);
   zk = z;
end    

% plotting response
figure(1)
time = (0:NT);
subplot(2,1,1)
plot(time,x(1,:),'r.-','LineWidth',.7) 
hold on
plot(time,x(2,:),'k.-','LineWidth',.7)
hold on
stairs(time,x_ref(1,1:(NT+1)), 'b--', 'LineWidth',.7)
plot(time,x_predicted(1,:), 'm.-', 'LineWidth', .7)
legend('$x_1$','$x_2$','$x_{reference}$','$x_{predicted}$','Interpreter','latex');
%axis ([0 50-10 10])
xlabel('$k$','Interpreter','latex');ylabel('$\textbf{x}_{k}$','Interpreter','latex');
grid on
ax = gca;
set(gca,'xtick',[0:5:NT])
set(gca,'ytick',(-2*max(x_min(:))):10:(1.5*max(x(1,:))))
ax.GridAlpha = 1
ax.GridLineStyle = ':'
subplot(2,1,2)
stairs(time(1:end-1),u,'r.-','LineWidth',.7)
hold on
stairs(time(1:NT),u_predicted(1,:), 'b.-', 'LineWidth', .7)
%axis([0 50 -10 0])
legend('$u$','$u_{predicted}$','Interpreter','latex');
xlabel('$k$','Interpreter','latex');ylabel('${u}_{k}$','Interpreter','latex');
grid on
ax = gca;
ylim([2*umin 2*umax])
%set(gca,'xtick',[0:5:NT])
%set(gca,'ytick',(-2):.5:(2))
ax.GridAlpha = 1
ax.GridLineStyle = ':'
print -dsvg lmpc1




