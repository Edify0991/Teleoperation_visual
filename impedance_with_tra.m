startup_rvc;
mdl_puma560;
%T0 = transl(0.4, 0.2, 0) * trotx(pi);
%T1 = transl(-0.4, 0.2, 0.3) * troty(pi/2) * trotz(-pi/2);
T0 = p560.fkine(qn);
T1 = p560.fkine(qr);
Ts = ctraj(T0, T1, 50);
q = p560.ikine(Ts);
p560.plot(q);

hold on;
M=1;  B=10;  K=30;
a=zeros(50,2);  b=5*ones(50,1); c=zeros(50,3); d=zeros(150,6);
Fe=[a,b,c;d];     %产生的200行6列的矩阵,且只在前半段时间z轴方向有向下的力，后50行无力，后期得改为六维力的数据
Fxe=Fe(:,1);Fye=Fe(:,2);Fze=Fe(:,3);Txe=Fe(:,4);Tye=Fe(:,5);Tze=Fe(:,6);