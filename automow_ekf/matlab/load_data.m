clear; clc; close all;

%% Load Models

clear F G;

importfile('models.csv');

F = zeros(7,7,length(data(:,1)));
G = zeros(7,7,length(data(:,1)));

for kk=1:length(data(:,1))
    for ii=1:7
       for jj=1:7
          F(ii, jj, kk) = data(kk, 7*(ii-1)+jj);
          G(ii, jj, kk) = data(kk, 7*(ii-1)+jj+49);
       end
    end
end

clear kk ii jj data textdata;


%% Load States

clear easting northing heading left_radius ...
      right_radius wheel_base_length easting_bias northing_bias ...
      heading_bias P;

importfile('states.csv');

easting = data(:,1);
northing = data(:,2);
heading = data(:,3);
left_radius = data(:,4);
right_radius = data(:,5);
wheel_base_length = data(:,6);
heading_bias = data(:,7);

P = zeros(7,7,length(data(:,1)));

for kk=1:length(data(:,1))
    for ii=1:7
       for jj=1:7
          P(ii, jj, kk) = data(kk, 7*(ii-1)+jj+7);
       end
    end
end

% clear kk ii jj data textdata;

%% Load AHRS data

clear ahrs ahrs_wrapped ahrs_prediction ahrs_inno ahrs_inno_wrapped ...
      ahrs_S ahrs_S_inv ahrs_K P_ahrs;

importfile('ahrs.csv');

ahrs = data(:,1);
ahrs_wrapped = data(:,2);
ahrs_prediction = data(:,3);
ahrs_inno = data(:,4);
ahrs_inno_wrapped = data(:,5);
ahrs_S = data(:,6);
ahrs_S_inv = data(:,7);
ahrs_K = zeros(7, length(data(:,1)));
for ii=1:7
    ahrs_K(ii,:) = data(:,ii+7);
end

P_ahrs = zeros(7,7,length(data(:,1)));

for kk=1:length(data(:,1))
    for ii=1:7
       for jj=1:7
          P_ahrs(ii, jj, kk) = data(kk, 9*(ii-1)+jj+14);
       end
    end
end

clear kk ii jj data textdata;

%%
% Plot ahrs with P

% figure(3);
% subplot(3,1,1);
% plot(squeeze(P(3,3,:)));
% subplot(3,1,2);
% plot(ahrs);
% subplot(3,1,3);
% plot(easting);

figure(4);
subplot(3,3,1)
plot(squeeze(P(1,1,:)))
subplot(3,3,2)
plot(squeeze(P(2,2,:)))
subplot(3,3,3)
plot(squeeze(P(3,3,:)))
subplot(3,3,4)
plot(squeeze(P(4,4,:)))
subplot(3,3,5)
plot(squeeze(P(5,5,:)))
subplot(3,3,6)
plot(squeeze(P(6,6,:)))
subplot(3,3,7:9)
plot(squeeze(P(7,7,:)))

%
% Plot AHRS wrapping

% figure(1);
% plot(ahrs, 'r');
% hold on;
% plot(ahrs_wrapped, 'b--');
% 
% figure(2);
% plot(ahrs_inno, 'r');
% hold on;
% plot(ahrs_inno_wrapped, 'b--');

%% Load Inputs and Time

clear left_input right_input time delta_time;

importfile('inputs.csv');

left_input = data(:,1);
right_input = data(:,2);
time = data(:,3);
delta_time = data(:,4);

clear kk ii jj data textdata;