clear; clc; close all;

%% Load Models

clear F G;

importfile('models.csv');

F = zeros(9,9,length(data(:,1)));
G = zeros(9,9,length(data(:,1)));

for kk=1:length(data(:,1))
    for ii=1:9
       for jj=1:9
          F(ii, jj, kk) = data(kk, 9*(ii-1)+jj);
          G(ii, jj, kk) = data(kk, 9*(ii-1)+jj+81);
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
easting_bias = data(:,7);
northing_bias = data(:,8);
heading_bias = data(:,9);

P = zeros(9,9,length(data(:,1)));

for kk=1:length(data(:,1))
    for ii=1:9
       for jj=1:9
          P(ii, jj, kk) = data(kk, 9*(ii-1)+jj+9);
       end
    end
end

clear kk ii jj data textdata;

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
ahrs_K = zeros(9, length(data(:,1)));
for ii=1:9
    ahrs_K(ii,:) = data(:,ii+7);
end

P_ahrs = zeros(9,9,length(data(:,1)));

for kk=1:length(data(:,1))
    for ii=1:9
       for jj=1:9
          P_ahrs(ii, jj, kk) = data(kk, 9*(ii-1)+jj+16);
       end
    end
end

clear kk ii jj data textdata;

%%
% Plot ahrs with P

figure(3);
subplot(3,1,1);
plot(squeeze(P_ahrs(3,3,:)));
subplot(3,1,2);
plot(ahrs);
subplot(3,1,3);
plot(easting);

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