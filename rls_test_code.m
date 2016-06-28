% Testfile for rls_const
% 2016/6/28 Yoshi Ri

clear all;
close all;
clc;

%% answer
a1 = -1.5;
a2 = 0.7;
b1 = 1.0;
b2 = 0.5;
Answer = [a1;a2;b1;b2];
%% setpu
Len = 100;
Input = 1;

x = zeros(Len,1);
v = 0.15 * (rand(Len,1) - 0.5);
u = Input * rand(Len,1);
est = zeros(Len,4);

%% estimation
n = size(Answer,1);
alpha = 10;
Est_0 = zeros(n,1)
estimater = rls_const(2);
rls_const(zeros(n,1),1000*eye(n),0.9999);



x(1) = 0; x(2) = 0;
y(1:2) = x(1:2) + v(1:2);

for i = 3:Len
    x(i) = - a1 * x(i-1) - a2 * x(i-2) + b1 * u(i-1) + b2 * u(i-2);
    y(i) = x(i) + v(i);
    Zn = [y(i-1); y(i-2); u(i-1); u(i-2)];
    Yn = [y(i)];
    estimater.estimate(Yn,Zn);
    est(i,:) = ( estimater.Theta )';
end

%%
plot(est);