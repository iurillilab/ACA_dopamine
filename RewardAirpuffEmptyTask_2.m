function RewardAirpuffEmptyTask

global BpodSystem

%% Assert modules
BpodSystem.assertModule({'RotaryEncoder', 'ValveModule'}, [1 1]);

%% Define parameters and trial structure
S = BpodSystem.ProtocolSettings;

if isempty(fieldnames(S))
    S.GUI.MaxTrials = 300;

    % Timing
    S.GUI.ITI_Mean = 18;
    S.GUI.ITI_Min = 8;
    S.GUI.ITI_Max = 28;

    % Reward
    S.GUI.RewardValve = 1;
    S.GUI.SmallRewardAmount_uL = 4;
    S.GUI.LargeRewardAmount_uL = 16;
    S.GUI.RewardValveTime = 0.05;
    S.GUI.RewardCollectionWindow = 10000;

    % Licking
    S.GUI.LickPort = 1;

    % Airpuff / Valve Module
    S.GUI.SafeExhaustValve = 1;  % Valve open by default
    S.GUI.AirpuffValve = 2;      % Valve open only during airpuff
    S.GUI.AirpuffDuration = 2;

    % Continuous sync for photometry
    S.GUI.SyncBNC = 1;
    S.GUI.SyncPulseWidth = 0.05;
    S.GUI.SyncFrequency = 10;

    S.GUIPanels.ITI = {'ITI_Mean', 'ITI_Min', 'ITI_Max'};
    S.GUIPanels.Reward = {'RewardValve', 'SmallRewardAmount_uL', 'LargeRewardAmount_uL', 'RewardCollectionWindow'};
    S.GUIPanels.Airpuff = {'SafeExhaustValve', 'AirpuffValve', 'AirpuffDuration'};
    S.GUIPanels.Sync = {'SyncBNC', 'SyncPulseWidth', 'SyncFrequency'};
end

BpodParameterGUI('init', S);

MaxTrials = S.GUI.MaxTrials;
TrialTypes = GeneratePseudoRandomTrials(MaxTrials); % 1=Reward, 2=Airpuff, 3=Empty

%% Setup modules
REM = RotaryEncoderModule(BpodSystem.ModuleUSB.RotaryEncoder1);
REM.wrapMode = 'Bipolar';
REM.sendThresholdEvents = 'off';

V = ValveDriverModule(BpodSystem.ModuleUSB.ValveModule1);

% Make sure safe/exhaust valve is open before the first trial starts
try
    V.isOpen = ValveVector(S.GUI.SafeExhaustValve);
catch ME
    warning('Could not set safe/exhaust valve at startup: %s', ME.message);
end

%% Initialize data fields
BpodSystem.Data.TrialTypes = [];
BpodSystem.Data.TrialTypeLabel = {};

BpodSystem.Data.RewardSizeLabel = {};
BpodSystem.Data.RewardAmount_uL = [];
BpodSystem.Data.RewardValveTimeUsed = [];

BpodSystem.Data.RewardOnset = [];
BpodSystem.Data.RewardOffset = [];
BpodSystem.Data.RewardCollectionEnd = [];

BpodSystem.Data.AirpuffOnset = [];
BpodSystem.Data.AirpuffOffset = [];

BpodSystem.Data.TotalLicks = [];
BpodSystem.Data.LickTimes = {};

BpodSystem.Data.EncoderData = {};
BpodSystem.Data.TrialSettings = {};

BpodSystem.Data.SyncPulseOn = {};
BpodSystem.Data.SyncPulseOff = {};

%% Main trial loop
TaskVis = InitTaskVisualization(MaxTrials, TrialTypes);

REM.startUSBStream;

for currentTrial = 1:MaxTrials

    S = BpodParameterGUI('sync', S);

    trialType = TrialTypes(currentTrial);
    trialLabel = TrialTypeToString(trialType);

    fprintf('\nTrial %d/%d | Type: %s\n', currentTrial, MaxTrials, trialLabel);

    UpdateTaskVisualizationStart(TaskVis, currentTrial, MaxTrials, trialType, trialLabel);

    lickInEvent = sprintf('Port%dIn', S.GUI.LickPort);

    BpodSystem.Data.TrialTypes(currentTrial) = trialType;
    BpodSystem.Data.TrialTypeLabel{currentTrial} = trialLabel;

    ITIDelay = generateRandomDelay(S.GUI.ITI_Mean, S.GUI.ITI_Min, S.GUI.ITI_Max);

    % ---------------- Valve Module byte messages ----------------
    % Message 1: safe/exhaust valve open, all others closed
    % Message 2: airpuff valve open, safe/exhaust valve closed
    % Message 3: all valves closed
    safeExhaustByte = ValveByte(S.GUI.SafeExhaustValve);
    airpuffByte = ValveByte(S.GUI.AirpuffValve);
    allClosedByte = uint8(0);

    LoadSerialMessages('ValveModule1', { ...
        ['B' safeExhaustByte], ...
        ['B' airpuffByte], ...
        ['B' allClosedByte]});

    % ---------------- Reward size randomization ----------------
    rewardAmount_uL = NaN;
    rewardValveTime = NaN;
    rewardSizeLabel = '';

    if trialType == 1
        if rand < 0.5
            rewardAmount_uL = S.GUI.SmallRewardAmount_uL;
            rewardSizeLabel = 'Small';
        else
            rewardAmount_uL = S.GUI.LargeRewardAmount_uL;
            rewardSizeLabel = 'Large';
        end

        try
            rewardValveTime = GetValveTimes(rewardAmount_uL, S.GUI.RewardValve);
        catch
            rewardValveTime = S.GUI.RewardValveTime * ...
                (rewardAmount_uL / max(1, S.GUI.SmallRewardAmount_uL));
        end
    end

    BpodSystem.Data.RewardSizeLabel{currentTrial} = rewardSizeLabel;
    BpodSystem.Data.RewardAmount_uL(currentTrial) = rewardAmount_uL;
    BpodSystem.Data.RewardValveTimeUsed(currentTrial) = rewardValveTime;

    % ---------------- Build state machine ----------------
    sma = NewStateMachine();

    % ---------------- Continuous 10 Hz sync global timer ----------------
    syncChannel = sprintf('BNC%d', S.GUI.SyncBNC);
    syncPeriod = 1 / S.GUI.SyncFrequency;
    syncInterval = syncPeriod - S.GUI.SyncPulseWidth;

    sma = SetGlobalTimer(sma, ...
        'TimerID', 1, ...
        'Duration', S.GUI.SyncPulseWidth, ...
        'OnsetDelay', 0.0001, ...
        'Channel', syncChannel, ...
        'OnsetValue', 1, ...
        'OffsetValue', 0, ...
        'Loop', 1, ...
        'GlobalTimerEvents', 1, ...
        'LoopInterval', syncInterval);

    % At the beginning of every trial:
    % Start sync timer and force safe/exhaust valve open.
    sma = AddState(sma, 'Name', 'TrialStart', ...
        'Timer', 0.01, ...
        'StateChangeConditions', {'Tup', 'ResetEncoder'}, ...
        'OutputActions', {'GlobalTimerTrig', 1, 'ValveModule1', 1});

    sma = AddState(sma, 'Name', 'ResetEncoder', ...
        'Timer', 0, ...
        'StateChangeConditions', {'Tup', 'ITI'}, ...
        'OutputActions', {'RotaryEncoder1', 'Z'});

    sma = AddState(sma, 'Name', 'ITI', ...
        'Timer', ITIDelay, ...
        'StateChangeConditions', {lickInEvent, 'ITI', 'Tup', 'DeliverOutcome'}, ...
        'OutputActions', {});

    sma = AddState(sma, 'Name', 'DeliverOutcome', ...
        'Timer', 0.001, ...
        'StateChangeConditions', {'Tup', OutcomeStateName(trialType)}, ...
        'OutputActions', {});

    switch trialType

        case 1 % Reward

            sma = AddState(sma, 'Name', 'Reward', ...
                'Timer', rewardValveTime, ...
                'StateChangeConditions', {lickInEvent, 'RewardCollection', 'Tup', 'RewardCollection'}, ...
                'OutputActions', {'ValveState', S.GUI.RewardValve});

            % sma = AddState(sma, 'Name', 'RewardOff', ...
            %     'Timer', 0.002, ...
            %     'StateChangeConditions', {lickInEvent, 'RewardOff', 'Tup', 'RewardCollection'}, ...
            %     'OutputActions', {});

            sma = AddState(sma, 'Name', 'RewardCollection', ...
                'Timer', S.GUI.RewardCollectionWindow, ...
                'StateChangeConditions', {lickInEvent, 'EndTrial', 'Tup', 'EndTrial'}, ...
                'OutputActions', {});

        case 2 % Airpuff

            % During airpuff:
            % safe/exhaust valve closes and airpuff valve opens.
            sma = AddState(sma, 'Name', 'DeliverAirPuff', ...
                'Timer', S.GUI.AirpuffDuration, ...
                'StateChangeConditions', {'Tup', 'EndAirPuff'}, ...
                'OutputActions', {'ValveModule1', 2});

            % After airpuff:
            % airpuff valve closes and safe/exhaust valve opens again.
            sma = AddState(sma, 'Name', 'EndAirPuff', ...
                'Timer', 0.01, ...
                'StateChangeConditions', {'Tup', 'EndTrial'}, ...
                'OutputActions', {'ValveModule1', 1});

        case 3 % Empty

            sma = AddState(sma, 'Name', 'Empty', ...
                'Timer', 10, ...
                'StateChangeConditions', {'Tup', 'EndTrial'}, ...
                'OutputActions', {});

        otherwise
            error('Invalid trial');
    end

    % Keep safe/exhaust valve open at the end as well.
    sma = AddState(sma, 'Name', 'EndTrial', ...
        'Timer', 0.05, ...
        'StateChangeConditions', {'Tup', '>exit'}, ...
        'OutputActions', {'ValveModule1', 1});

    %% Send and run
    SendStateMachine(sma);
    RawEvents = RunStateMachine();

    if BpodSystem.Status.BeingUsed == 0
        SafeCloseValves(V, S.GUI.SafeExhaustValve);
        REM.stopUSBStream;
        return
    end

    if ~isempty(fieldnames(RawEvents))

        BpodSystem.Data = AddTrialEvents(BpodSystem.Data, RawEvents);
        BpodSystem.Data.TrialSettings{currentTrial} = S;

        % Read encoder data
        encoderDataThisTrial = REM.readUSBStream();
        BpodSystem.Data.EncoderData{currentTrial} = encoderDataThisTrial;

        UpdateTaskVisualizationEnd(TaskVis, currentTrial);

        % Extract trial events
        trialData = BpodSystem.Data.RawEvents.Trial{currentTrial};

        % Sync pulse timestamps from global timer
        if isfield(trialData.Events, 'GlobalTimer1_Start')
            BpodSystem.Data.SyncPulseOn{currentTrial} = trialData.Events.GlobalTimer1_Start;
        else
            BpodSystem.Data.SyncPulseOn{currentTrial} = [];
        end

        if isfield(trialData.Events, 'GlobalTimer1_End')
            BpodSystem.Data.SyncPulseOff{currentTrial} = trialData.Events.GlobalTimer1_End;
        else
            BpodSystem.Data.SyncPulseOff{currentTrial} = [];
        end

        % Licks
        lickTimes = [];
        if isfield(trialData.Events, lickInEvent)
            lickTimes = trialData.Events.(lickInEvent);
        end

        BpodSystem.Data.LickTimes{currentTrial} = lickTimes;
        BpodSystem.Data.TotalLicks(currentTrial) = numel(lickTimes);

        % Reward times
        rewardState = GetStateWindowSafe(trialData, 'Reward');
        if ~any(isnan(rewardState))
            BpodSystem.Data.RewardOnset(currentTrial) = rewardState(1);
            BpodSystem.Data.RewardOffset(currentTrial) = rewardState(2);
        else
            BpodSystem.Data.RewardOnset(currentTrial) = NaN;
            BpodSystem.Data.RewardOffset(currentTrial) = NaN;
        end

        % Reward collection end
        rewardCollectionState = GetStateWindowSafe(trialData, 'RewardCollection');
        if ~any(isnan(rewardCollectionState))
            BpodSystem.Data.RewardCollectionEnd(currentTrial) = rewardCollectionState(2);
        else
            BpodSystem.Data.RewardCollectionEnd(currentTrial) = NaN;
        end

        % Airpuff times
        airpuffState = GetStateWindowSafe(trialData, 'DeliverAirPuff');
        if ~any(isnan(airpuffState))
            BpodSystem.Data.AirpuffOnset(currentTrial) = airpuffState(1);
            BpodSystem.Data.AirpuffOffset(currentTrial) = airpuffState(2);
        else
            BpodSystem.Data.AirpuffOnset(currentTrial) = NaN;
            BpodSystem.Data.AirpuffOffset(currentTrial) = NaN;
        end

        SaveBpodSessionData;
    end

    HandlePauseCondition();

    if BpodSystem.Status.BeingUsed == 0
        SafeCloseValves(V, S.GUI.SafeExhaustValve);
        REM.stopUSBStream;
        return
    end
end

SafeCloseValves(V, S.GUI.SafeExhaustValve);
REM.stopUSBStream;

end

%% =========================================================
function TrialTypes = GeneratePseudoRandomTrials(MaxTrials)

baseSet = [1 2 3];
nBlocks = ceil(MaxTrials / 3);
TrialTypes = [];

for i = 1:nBlocks
    TrialTypes = [TrialTypes baseSet(randperm(3))]; %#ok<AGROW>
end

TrialTypes = TrialTypes(1:MaxTrials);

end

%% =========================================================
function name = TrialTypeToString(trialType)

switch trialType
    case 1
        name = 'Reward';
    case 2
        name = 'Airpuff';
    case 3
        name = 'Empty';
    otherwise
        name = 'Unknown';
end

end

%% =========================================================
function nextState = OutcomeStateName(trialType)

switch trialType
    case 1
        nextState = 'Reward';
    case 2
        nextState = 'DeliverAirPuff';
    case 3
        nextState = 'Empty';
    otherwise
        error('Invalid trial type');
end

end

%% =========================================================
function window = GetStateWindowSafe(trialData, stateName)

window = [NaN NaN];

if ~isfield(trialData, 'States')
    return
end

if ~isfield(trialData.States, stateName)
    return
end

st = trialData.States.(stateName);

if isempty(st)
    return
end

if any(isnan(st(1,:)))
    return
end

window = [st(1,1) st(end,2)];

end

%% =========================================================
function randomDelay = generateRandomDelay(meanDelay, minDelay, maxDelay)

while true
    randomDelay = exprnd(meanDelay);

    if randomDelay >= minDelay && randomDelay <= maxDelay
        break
    end
end

end

%% =========================================================
function byteValue = ValveByte(valveNumber)
% Convert a Valve Module valve number, 1-8, into the byte used by the B command.
%
% Valve 1 -> 1
% Valve 2 -> 2
% Valve 3 -> 4
% Valve 4 -> 8
%
% For one open valve:
% byteValue = 2^(valveNumber - 1)

if valveNumber < 1 || valveNumber > 8
    error('Valve number must be between 1 and 8.');
end

byteValue = uint8(2^(valveNumber - 1));

end

%% =========================================================
function valveState = ValveVector(openValve)
% Return a 1x8 vector for ValveDriverModule.isOpen.
% Only openValve is open; all other Valve Module valves are closed.

if openValve < 1 || openValve > 8
    error('Valve number must be between 1 and 8.');
end

valveState = zeros(1, 8);
valveState(openValve) = 1;

end

%% =========================================================
function SafeCloseValves(V, safeValve)
% Safety state:
% leave the safe/exhaust valve open and close all other Valve Module valves.

try
    V.isOpen = ValveVector(safeValve);
catch
end

end

%% =========================================================
function TaskVis = InitTaskVisualization(MaxTrials, TrialTypes)

TaskVis = struct();

TaskVis.Fig = figure( ...
    'Name', 'Bpod Task Monitor', ...
    'NumberTitle', 'off', ...
    'Color', 'w', ...
    'MenuBar', 'none', ...
    'ToolBar', 'figure');

TaskVis.StatusText = uicontrol( ...
    'Style', 'text', ...
    'Units', 'normalized', ...
    'Position', [0.05 0.90 0.90 0.07], ...
    'String', 'Waiting to start...', ...
    'FontSize', 16, ...
    'FontWeight', 'bold', ...
    'BackgroundColor', 'w', ...
    'HorizontalAlignment', 'center');

TaskVis.TrialText = uicontrol( ...
    'Style', 'text', ...
    'Units', 'normalized', ...
    'Position', [0.05 0.84 0.90 0.05], ...
    'String', '', ...
    'FontSize', 12, ...
    'BackgroundColor', 'w', ...
    'HorizontalAlignment', 'center');

TaskVis.AxTrialTypes = axes( ...
    'Parent', TaskVis.Fig, ...
    'Units', 'normalized', ...
    'Position', [0.08 0.58 0.86 0.20]);

hold(TaskVis.AxTrialTypes, 'on');

plot(TaskVis.AxTrialTypes, 1:MaxTrials, TrialTypes, 'ko', ...
    'MarkerFaceColor', [0.8 0.8 0.8], ...
    'MarkerSize', 5);

TaskVis.CurrentTrialMarker = plot(TaskVis.AxTrialTypes, NaN, NaN, 'ro', ...
    'MarkerFaceColor', 'r', ...
    'MarkerSize', 10);

ylim(TaskVis.AxTrialTypes, [0.5 3.5]);
xlim(TaskVis.AxTrialTypes, [1 MaxTrials]);
yticks(TaskVis.AxTrialTypes, [1 2 3]);
yticklabels(TaskVis.AxTrialTypes, {'Reward', 'Airpuff', 'Empty'});
xlabel(TaskVis.AxTrialTypes, 'Trial');
ylabel(TaskVis.AxTrialTypes, 'Trial type');
title(TaskVis.AxTrialTypes, 'Trial sequence');

drawnow;

end

%% =========================================================
function UpdateTaskVisualizationStart(TaskVis, currentTrial, MaxTrials, trialType, trialLabel)

if ~isfield(TaskVis, 'Fig') || ~isvalid(TaskVis.Fig)
    return
end

switch trialType
    case 1
        statusColor = [0.85 1.00 0.85]; % light green
    case 2
        statusColor = [1.00 0.85 0.85]; % light red
    case 3
        statusColor = [0.90 0.90 0.90]; % light gray
    otherwise
        statusColor = [1 1 1];
end

set(TaskVis.StatusText, ...
    'String', sprintf('ONGOING TRIAL: %s', trialLabel), ...
    'BackgroundColor', statusColor);

set(TaskVis.TrialText, ...
    'String', sprintf('Trial %d of %d', currentTrial, MaxTrials), ...
    'BackgroundColor', 'w');

set(TaskVis.CurrentTrialMarker, ...
    'XData', currentTrial, ...
    'YData', trialType);

drawnow;

end

%% =========================================================
function UpdateTaskVisualizationEnd(TaskVis, currentTrial)

if ~isfield(TaskVis, 'Fig') || ~isvalid(TaskVis.Fig)
    return
end

set(TaskVis.StatusText, ...
    'String', sprintf('Completed trial %d', currentTrial), ...
    'BackgroundColor', 'w');

drawnow;

end
