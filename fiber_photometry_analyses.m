clear all 
clc

data = import_ppd('E:\Fiber_Photometry\E451\E451-2026-05-29-102607.ppd'); %fiber photmetry data

processed_signal = preprocess_data(data, ...
    'signal', 'analog_1', ...
    'control', 'analog_2', ...
    'low_pass', 10, ...
    'normalisation', 'dF/F', ...
    'plot', true); %preprocessed photometry signals

load('E:\Fiber_Photometry\E451\RewardAirpuffEmptyTask_2\Session Data\E451_RewardAirpuffEmptyTask_2_20260529_102602.mat') %matlab data

%% photometry signal and digital data
% There is a problem because the Bpod signal is too fast compared to the sampling rate of the fiber photometry system. 
% The frequency is 10 Hz, but the signal lasts 1 ms, whereas photometry sampling occurs every 7.6923 ms. 
% This problem was solved for subsequent acquisitions by extending the pulses to 50 ms.
% For these data, however, the first and last TTL recorded by the photometry system were used to align the results.

fp_time = data.time / 1000; %data.time is sampled every 7.6923 ms, consistent with the fiber photometry acquisition rate of 130 Hz; fp_time is in seconds

figure
plot(fp_time,processed_signal)
hold on
plot(fp_time,data.digital_1)

fp_sync_idx = find(diff(data.digital_1) == 1) + 1;
fp_sync_time = fp_time(fp_sync_idx);

fp_sync = fp_sync_time - fp_sync_time(1); %fp_sync is sampled every 100ms, consistent with the 10 Hz signal (data.digital_1) that Bpod sends to the photometry system

bpod_sync_time= [];
for i = 1:SessionData.nTrials
    bpod_sync_time = [bpod_sync_time; SessionData.TrialStartTimestamp(i) + SessionData.SyncPulseOn{i}(:)];
end

bpod_sync = bpod_sync_time - bpod_sync_time(1); %bpod_sync is sampled every 100ms, consistent with the 10 Hz signal (SessionData.SyncPulseOn) that Bpod sends to the photometry system

%fp_sync e bpod_sync should be the same signal, the first recorded from the fp, the second recorded from the bpod

% Cut photometry signal at the first and last TTL recorded by the fiber photometry system 

fp_time_cut = fp_time(fp_sync_idx(1):fp_sync_idx(end));
processed_signal_cut = processed_signal(fp_sync_idx(1):fp_sync_idx(end));
digital_1_cut = data.digital_1(fp_sync_idx(1):fp_sync_idx(end));

fp_time_signal = fp_time_cut - fp_time_cut(1);

figure
plot(fp_time_signal,processed_signal_cut)
hold on
plot(fp_time_signal,digital_1_cut)

%% Create table trial-by-trial with Bpod + fiber photometry info

trial_start = SessionData.TrialStartTimestamp(:) - bpod_sync_time(1); 
trial_end = SessionData.TrialEndTimestamp(:)- bpod_sync_time(1); 
nTrials = SessionData.nTrials;

% Preallocate 
trial_id = (1:nTrials)';
trial_label = SessionData.TrialTypeLabel(:);
fp_time_from_trial_start = cell(nTrials,1);
fp_time_trial = cell(nTrials,1);
fp_signal_trial = cell(nTrials,1);
fp_digital_trial = cell(nTrials,1);

reward_on = NaN(nTrials,1);
reward_off = NaN(nTrials,1);
airpuff_on = NaN(nTrials,1);
airpuff_off = NaN(nTrials,1);
n_licks = NaN(nTrials,1);
lick_times = cell(nTrials,1);

reward_amount = NaN(nTrials,1);
reward_size = cell(nTrials,1);

Wheel_TimeFromTrialStart_s = cell(nTrials,1);
Wheel_Position = cell(nTrials,1);
Wheel_Velocity = cell(nTrials,1);
Wheel_AbsVelocity = cell(nTrials,1);

for i = 1:nTrials

    % Fiber photometry
    idx = fp_time_signal >= trial_start(i) & fp_time_signal < trial_end(i);
    fp_time_from_trial_start{i} = fp_time_signal(idx) - trial_start(i);
    fp_time_trial{i} = fp_time_signal(idx);
    fp_signal_trial{i} = processed_signal_cut(idx);

    % Bpod Event
    reward_on(i) = SessionData.RewardOnset(i)+trial_start(i);
    reward_off(i) = SessionData.RewardOffset(i)+trial_start(i);

    airpuff_on(i) = SessionData.AirpuffOnset(i)+trial_start(i);
    airpuff_off(i) = SessionData.AirpuffOffset(i)+trial_start(i);

    lick_times{i} = SessionData.LickTimes{i}+trial_start(i);
    n_licks(i) = numel(SessionData.LickTimes{i});

    reward_amount(i) = SessionData.RewardAmount_uL(i);
    reward_size{i} = SessionData.RewardSizeLabel{i};

    % Encoder Events
    enc=SessionData.EncoderData{i};
    wheel_t = enc.Times(:);
    wheel_pos = enc.Positions(:);

    % Wheel speed
    wheel_vel = [NaN; diff(wheel_pos) ./ diff(wheel_t)];
    wheel_absvel = abs(wheel_vel);
    wheel_absvel_on_fp = interp1(wheel_t, wheel_absvel, fp_time_from_trial_start{i}, 'linear', NaN);

    % Saving 
    Wheel_TimeFromTrialStart_s{i} = fp_time_from_trial_start{i};
    Wheel_Position{i} = interp1(wheel_t, wheel_pos, fp_time_from_trial_start{i}, 'linear', NaN);
    Wheel_Velocity{i} = interp1(wheel_t, wheel_vel, fp_time_from_trial_start{i}, 'linear', NaN);
    Wheel_AbsVelocity{i} = wheel_absvel_on_fp;

end

TrialTable = table( ...
    trial_id, ...
    trial_label, ...
    trial_start, ...
    trial_end, ...
    reward_size, ...
    reward_amount, ...
    reward_on, ...
    reward_off, ...
    airpuff_on, ...
    airpuff_off, ...
    n_licks, ...
    lick_times, ...
    fp_time_trial, ...
    fp_time_from_trial_start, ...
    fp_signal_trial, ...
    Wheel_TimeFromTrialStart_s, ...
    Wheel_Position, ...
    Wheel_Velocity, ...
    Wheel_AbsVelocity);

TrialTable.Properties.VariableNames = { ...
    'Trial', ...
    'TrialLabel', ...
    'TrialStart_s', ...
    'TrialEnd_s', ...
    'RewardSize', ...
    'RewardAmount_uL', ...
    'RewardOnset_s', ...
    'RewardOffset_s', ...
    'AirpuffOnset_s', ...
    'AirpuffOffset_s', ...
    'NLicks', ...
    'LickTimes_s', ...
    'FP_Time_s', ...
    'FP_TimeFromTrialStart_s', ...
    'FP_Signal', ...
    'Wheel_TimeFromTrialStart_s', ...
    'Wheel_Position', ...
    'Wheel_Velocity', ...
    'Wheel_AbsVelocity'};

%% Airpuff trials 

pre_time = 5;  %sec
post_time = 10;  %sec

t_common = -pre_time : 1/data.sampling_rate : post_time;

% Airpuff trials
is_airpuff = strcmp(TrialTable.TrialLabel, 'Airpuff');
AirpuffTrials = TrialTable(is_airpuff, :);

nAirpuff = height(AirpuffTrials);

% Photometry signal aligned with the airpuff onset
airpuff_mat = NaN(nAirpuff, length(t_common));
for i = 1:nAirpuff
    t_airpuff = AirpuffTrials.FP_TimeFromTrialStart_s{i} - (AirpuffTrials.AirpuffOnset_s(i)-AirpuffTrials.TrialStart_s(i));
    airpuff_mat(i,:) = interp1(t_airpuff, AirpuffTrials.FP_Signal{i}, t_common, 'linear', NaN);
end

% Mean e SD
airpuff_mean = mean(airpuff_mat, 1, 'omitnan');
airpuff_sd = std(airpuff_mat, 0, 1, 'omitnan');

upper = airpuff_mean + airpuff_sd;
lower = airpuff_mean - airpuff_sd;

valid = ~isnan(t_common) & ~isnan(upper) & ~isnan(lower);

figure
hold on

x_fill = [t_common(valid), fliplr(t_common(valid))];
y_fill = [upper(valid), fliplr(lower(valid))];

h = fill(x_fill, y_fill, [0.7 0.7 0.7]);
set(h, 'EdgeColor', 'none');
set(h, 'FaceAlpha', 0.5);

plot(t_common, airpuff_mean, 'k', 'LineWidth', 2)

xline(0, '--r', 'Airpuff onset', 'LineWidth', 1.5)

xlabel('Time from airpuff onset (s)')
ylabel('dF/F')
title(sprintf('Airpuff aligned photometry, mean ± SD, n = %d', nAirpuff))

box off

%%

all_fp = [];
all_vel = [];
all_fp_times = [];

is_empty = strcmp(TrialTable.TrialLabel, 'Empty');
EmptyTrials = TrialTable(is_empty, :);

nEmptyTrials = height(EmptyTrials);

for i = 1:nEmptyTrials
    fp = EmptyTrials.FP_Signal{i};
    vel = EmptyTrials.Wheel_AbsVelocity{i}(:);
    time = EmptyTrials.FP_Time_s{i}(:);

    valid = ~isnan(fp) & ~isnan(vel);

    all_fp_times = [all_fp_times, time(valid)];
    all_fp = [all_fp; fp(valid)];
    all_vel = [all_vel; vel(valid)];
end

% Velocity-photometry correlation in empty trials 
[r, p] = corr(all_vel, all_fp, 'Rows', 'complete');
fprintf('Corr velocità-fotometria: r = %.3f, p = %.3g\n', r, p);

figure
s = scatter(all_vel, all_fp, 8, 'filled');
s.MarkerFaceColor = [0.2 0.2 0.2];
s.MarkerFaceAlpha = 0.12;
s.MarkerEdgeAlpha = 0.12;

hold on

mdl = fitlm(all_vel, all_fp);
xfit = linspace(min(all_vel), max(all_vel), 100);
yfit = predict(mdl, xfit');

plot(xfit, yfit, ...
    'Color', [0.1 0.25 0.8], ... 
    'LineWidth', 2.5);

xlabel('Wheel velocity')
ylabel('Photometry signal')
title(sprintf('Empty trials: velocity vs photometry, r = %.3f, p = %.2g', r, p))

box off

%
figure('Color','w');
hold on;

yyaxis left
plot(all_fp_times, all_fp, 'b', 'LineWidth', 1.2);
ylabel('Fiber photometry signal');
ylim([-5 5]); 

yyaxis right
plot(all_fp_times, all_vel, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.2);
ylabel('Rotary encoder');
ylim([0 200]); 

xlabel('Time from event (s)');
xlim([0 15]);

title('Empty trials: photometry and rotary encoder');
box off;





