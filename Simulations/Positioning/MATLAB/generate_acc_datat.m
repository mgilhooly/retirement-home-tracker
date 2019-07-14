clear;

dt = 0.01;
t=0:dt:300;

accX_var_good = 0.0007; % (m/s^2)^2
accX_var_worst = 0.001; % (m/s^2)^2

accX_good_noise = randn(size(t))*sqrt(accX_var_good);
accX_worst_noise = randn(size(t))*sqrt(accX_var_worst);

accX_basesignal = sin(0.3*t) + 0.5*sin(0.04*t);

accX_ref = accX_basesignal;
velX_ref = cumsum(accX_ref)*dt;
distX_ref = cumsum(velX_ref)*dt;

accX_good = accX_basesignal + accX_good_noise;
velX_good = cumsum(accX_good)*dt;
distX_good = cumsum(velX_good)*dt;

accX_worst = accX_basesignal + accX_worst_noise;
velX_worst = cumsum(accX_worst)*dt;
distX_worst = cumsum(velX_worst)*dt;

subplot(3,3,1);
plot(t, accX_ref);
hold on;
plot(t, accX_good);
plot(t, accX_worst);
hold off;
grid minor;
legend('ref', 'good', 'worst');
title('AccX');

subplot(3,3,4);
plot(t, velX_ref);
hold on;
plot(t, velX_good);
plot(t, velX_worst);
hold off;
grid minor;
legend('ref', 'good', 'worst');
title('VelX');

subplot(3,3,7);
plot(t, distX_ref);
hold on;
plot(t, distX_good);
plot(t, distX_worst);
hold off;
grid minor;
legend('ref', 'good', 'worst');
title('DistX');

accY_var_good = 0.0007; % (m/s^2)^2
accY_var_worst = 0.001; % (m/s^2)^2

accY_good_noise = randn(size(t))*sqrt(accY_var_good);
accY_worst_noise = randn(size(t))*sqrt(accY_var_worst);

accY_basesignal = 0.5*square(t);

accY_ref = accY_basesignal;
velY_ref = cumsum(accY_ref)*dt;
distY_ref = cumsum(velY_ref)*dt;

accY_good = accY_basesignal + accY_good_noise;
velY_good = cumsum(accY_good)*dt;
distY_good = cumsum(velY_good)*dt;

accY_worst = accY_basesignal + accY_worst_noise;
velY_worst = cumsum(accY_worst)*dt;
distY_worst = cumsum(velY_worst)*dt;

subplot(3,3,2);
plot(t, accY_ref);
hold on;
plot(t, accY_good);
plot(t, accY_worst);
hold off;
grid minor;
legend('ref', 'good', 'worst');
title('AccY');

subplot(3,3,5);
plot(t, velY_ref);
hold on;
plot(t, velY_good);
plot(t, velY_worst);
hold off;
grid minor;
legend('ref', 'good', 'worst');
title('VelY');

subplot(3,3,8);
plot(t, distY_ref);
hold on;
plot(t, distY_good);
plot(t, distY_worst);
hold off;
grid minor;
legend('ref', 'good', 'worst');
title('DistY');

figure(2)
subplot(3,1,1);
plot(t, accY_ref);
hold on;
plot(t, accY_good);
plot(t, accY_worst);
hold off;
grid minor;
legend('ref', 'good', 'worst');
title('AccY');

subplot(3,1,2);
plot(t, velY_ref);
hold on;
plot(t, velY_good);
plot(t, velY_worst);
hold off;
grid minor;
legend('ref', 'good', 'worst');
title('VelY');

subplot(3,1,3);
plot(t, distY_ref);
hold on;
plot(t, distY_good);
plot(t, distY_worst);
hold off;
grid minor;
legend('ref', 'good', 'worst');
title('DistY');

accZ_var_good = 0.0007; % (m/s^2)^2
accZ_var_worst = 0.001; % (m/s^2)^2

accZ_good_noise = randn(size(t))*sqrt(accZ_var_good);
accZ_worst_noise = randn(size(t))*sqrt(accZ_var_worst);

accZ_basesignal = 0.5*sawtooth(t);

accZ_ref = accZ_basesignal;
velZ_ref = cumsum(accZ_ref)*dt;
distZ_ref = cumsum(velZ_ref)*dt;

accZ_good = accZ_basesignal + accZ_good_noise;
velZ_good = cumsum(accZ_good)*dt;
distZ_good = cumsum(velZ_good)*dt;

accZ_worst = accZ_basesignal + accZ_worst_noise;
velZ_worst = cumsum(accZ_worst)*dt;
distZ_worst = cumsum(velZ_worst)*dt;

figure(1)
subplot(3,3,3);
plot(t, accZ_ref);
hold on;
plot(t, accZ_good);
plot(t, accZ_worst);
hold off;
grid minor;
legend('ref', 'good', 'worst');
title('AccY');

subplot(3,3,6);
plot(t, velZ_ref);
hold on;
plot(t, velZ_good);
plot(t, velZ_worst);
hold off;
grid minor;
legend('ref', 'good', 'worst');
title('VelZ');

subplot(3,3,9);
plot(t, distZ_ref);
hold on;
plot(t, distZ_good);
plot(t, distZ_worst);
hold off;
grid minor;
legend('ref', 'good', 'worst');
title('DistZ');

MSE_distX = -(distX_ref - distX_good);
MSE_distY = -(distY_ref - distY_good);
MSE_distZ = -(distZ_ref - distZ_good);

min(MSE_distX)
max(MSE_distX)
mean(MSE_distX)

min(MSE_distY)
max(MSE_distY)
mean(MSE_distY)

min(MSE_distZ)
max(MSE_distZ)
mean(MSE_distZ)
% 
% MSE_distX_w = mean((distX_ref - distX_worst).^2)
% MSE_distY_w = mean((distY_ref - distY_worst).^2)
% MSE_distZ_w = mean((distZ_ref - distZ_worst).^2)


% dataX_out = [t; accX_ref;   velX_ref;   distX_ref;...
%                 accX_good;  velX_good;  distX_good;...
%                 accX_worst; velX_worst; distX_worst];
% csvwrite('accX_data.csv', dataX_out.')
% 
% dataY_out = [t; accY_ref;   velY_ref;   distY_ref;...
%                 accY_good;  velY_good;  distY_good;...
%                 accY_worst; velY_worst; distY_worst];
% csvwrite('accY_data.csv', dataY_out.')
% 
% dataZ_out = [t; accZ_ref;   velZ_ref;   distZ_ref;...
%                 accZ_good;  velZ_good;  distZ_good;...
%                 accZ_worst; velZ_worst; distZ_worst];
% csvwrite('accZ_data.csv', dataZ_out.')