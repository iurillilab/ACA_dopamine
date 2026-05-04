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

    % Airpuff
    S.GUI.AirpuffDuration = 2;
    S.GUI.CloseExhaustDuration = 0.10;

    % Continuous sync for photometry
    S.GUI.SyncBNC = 1;              % BNC output carrying 10 Hz sync pulses
    S.GUI.SyncPulseWidth = 0.001;   % 1 ms TTL pulse
    S.GUI.SyncFrequency = 10;       % Hz

    S.GUIPanels.ITI = {'ITI_Mean', 'ITI_Min', 'ITI_Max'};
    S.GUIPanels.Reward = {'RewardValve', 'SmallRewardAmount_uL', 'LargeRewardAmount_uL', 'RewardCollectionWindow'};
    S.GUIPanels.Airpuff = {'AirpuffDuration', 'CloseExhaustDuration'};
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
BpodSystem.Data.TrialSettings = [];

BpodSystem.Data.SyncPulseOn = {};
BpodSystem.Data.SyncPulseOff = {};
%% Main trial loop
REM.startUSBStream;

for currentTrial = 1:MaxTrials
    
    S = BpodParameterGUI('sync', S);
    trialType = TrialTypes(currentTrial);
    lickInEvent = sprintf('Port%dIn', S.GUI.LickPort);

    BpodSystem.Data.TrialTypes(currentTrial) = trialType;
    BpodSystem.Data.TrialTypeLabel{currentTrial} = TrialTypeToString(trialType);

    ITIDelay = generateRandomDelay(S.GUI.ITI_Mean, S.GUI.ITI_Min, S.GUI.ITI_Max);

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
            rewardValveTime = S.GUI.RewardValveTime * (rewardAmount_uL / max(1, S.GUI.SmallRewardAmount_uL));
        end
    end

    BpodSystem.Data.RewardSizeLabel{currentTrial} = rewardSizeLabel;
    BpodSystem.Data.RewardAmount_uL(currentTrial) = rewardAmount_uL;
    BpodSystem.Data.RewardValveTimeUsed(currentTrial) = rewardValveTime;

    % ---------------- Build state machine ----------------
    sma = NewStateMachine();

    % ---------------- Continuous 10 Hz sync global timer ----------------
    % Sends periodic TTL pulses to photometry / external systems.
    % GlobalTimer1_Start and GlobalTimer1_End are timestamped in RawEvents.
    syncChannel = sprintf('BNC%d', S.GUI.SyncBNC); %'BNC1'
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

    sma = AddState(sma, 'Name', 'TrialStart', ...
        'Timer', 0.01, ...
        'StateChangeConditions', {'Tup', 'ResetEncoder'}, ...
        'OutputActions', {'GlobalTimerTrig', 1});

    sma = AddState(sma, 'Name', 'ResetEncoder', ...
        'Timer', 0, ...
        'StateChangeConditions', {'Tup', 'ITI'}, ...
        'OutputActions', {'RotaryEncoder1', 'Z'});

    sma = AddState(sma, 'Name', 'ITI', ...
        'Timer', ITIDelay, ...
        'StateChangeConditions', {'Tup', 'DeliverOutcome'}, ...
        'OutputActions', {});

    sma = AddState(sma, 'Name', 'DeliverOutcome', ...
        'Timer', 0.001, ...
        'StateChangeConditions', {'Tup', OutcomeStateName(trialType)}, ...
        'OutputActions', {});

    switch trialType
        case 1 % Reward
            sma = AddState(sma, 'Name', 'Reward', ...
                'Timer', rewardValveTime, ...
                'StateChangeConditions', {'Tup', 'RewardOff'}, ...
                'OutputActions', {'ValveState', S.GUI.RewardValve});

            sma = AddState(sma, 'Name', 'RewardOff', ...
                'Timer', 0.002, ...
                'StateChangeConditions', {'Tup', 'RewardCollection'}, ...
                'OutputActions', {});

            sma = AddState(sma, 'Name', 'RewardCollection', ...
                'Timer', S.GUI.RewardCollectionWindow, ...
                'StateChangeConditions', {lickInEvent, 'EndTrial', 'Tup', 'EndTrial'}, ...
                'OutputActions', {});

        case 2 % Airpuff
            sma = AddState(sma, 'Name', 'CloseExhaustValve', ...
                'Timer', S.GUI.CloseExhaustDuration, ...
                'StateChangeConditions', {'Tup', 'DeliverAirPuff'}, ...
                'OutputActions', {'ValveModule1', 1});

            sma = AddState(sma, 'Name', 'DeliverAirPuff', ...
                'Timer', S.GUI.AirpuffDuration, ...
                'StateChangeConditions', {'Tup', 'EndAirPuff'}, ...
                'OutputActions', {'ValveModule1', 2});

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

    sma = AddState(sma, 'Name', 'EndTrial', ...
        'Timer', 0.05, ...
        'StateChangeConditions', {'Tup', '>exit'}, ...
        'OutputActions', {});

    %% Send and run
    SendStateMachine(sma);
    RawEvents = RunStateMachine();

    if BpodSystem.Status.BeingUsed == 0
        SafeCloseValves(V);
        REM.stopUSBStream;
        return
    end

    if ~isempty(fieldnames(RawEvents))
        BpodSystem.Data = AddTrialEvents(BpodSystem.Data, RawEvents);
        BpodSystem.Data.TrialSettings(currentTrial) = S;

        % Read encoder data
        BpodSystem.Data.EncoderData{currentTrial} = REM.readUSBStream();

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
        SafeCloseValves(V);
        REM.stopUSBStream;
        return
    end
end

SafeCloseValves(V);
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
        nextState = 'CloseExhaustValve';
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
function SafeCloseValves(V)
for idxValve = 1:8
    try
        V.closeValve(idxValve);
    catch
    end
end
try
    V.openValve(1); % same convention as previous script
catch
end
end
