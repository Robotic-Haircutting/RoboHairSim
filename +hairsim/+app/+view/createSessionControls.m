function ui = createSessionControls(parentFigure, callbacks, config)
%CREATESESSIONCONTROLS The control column every interactive entry shares:
%   Robot Setup, status line, Recording and (optionally) Hair Modeling,
%   plus an empty slot (ui.centerPanel) the entry fills with its own panel
%   (hairsim.app.view.createAutoPanel / createTeleopPanel).
%   Pure UI constructor: handles only, no session state.
%
%   callbacks: RobotSetupChanged, ToggleRecording (function handles).
%   config (optional): CenterTitle (slot title), BasePlacementOptions
%   (struct array id/label, default fixed/ceiling/rail135/150/180),
%   HairModelingConfig + HairModelingCallbacks (both needed for the Hair
%   Modeling panel; see hairsim.hair.createModelingControlPanel).

arguments
    parentFigure
    callbacks (1, 1) struct
    config (1, 1) struct = struct()
end

for name = ["RobotSetupChanged", "ToggleRecording"]
    if ~isfield(callbacks, name) || ~isa(callbacks.(name), 'function_handle')
        error("hairsim:MissingControlCallback", "Missing control callback: %s", name);
    end
end
basePlacementOptions = configValue(config, 'BasePlacementOptions', struct( ...
    'id', {"fixed", "ceiling", "rail180"}, ...
    'label', {"Fixed", "Ceiling", "Rail"}));   % Rail starts at 180 deg; Auto picks its stations, Teleop slides with [ ]

controlPanel = uipanel(parentFigure, 'Units', 'normalized', 'Position', [0.80, 0.08, 0.18, 0.82], ...
    'Title', 'Control', 'BackgroundColor', 'w');
operationPanel = uipanel(controlPanel, 'Units', 'normalized', ...
    'Position', [0.06, 0.42, 0.88, 0.57], 'BorderType', 'none', 'BackgroundColor', 'w');

uicontrol(operationPanel, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.0, 0.955, 0.80, 0.04], 'String', 'Robot Setup', ...
    'BackgroundColor', 'w', 'FontWeight', 'bold', 'HorizontalAlignment', 'left');
robotSetupPopup = uicontrol(operationPanel, 'Style', 'popupmenu', 'Units', 'normalized', ...
    'Position', [0.0, 0.90, 1.0, 0.05], 'String', {basePlacementOptions.label}, ...
    'Value', 1, 'Callback', callbacks.RobotSetupChanged, 'Tag', 'hairsimRobotSetupPopup');

statusText = uicontrol(operationPanel, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.0, 0.85, 1.0, 0.04], 'String', 'Status: idle', ...
    'BackgroundColor', 'w', 'FontWeight', 'bold', 'HorizontalAlignment', 'left', 'Tag', 'hairsimStatusText');

recordPanel = uipanel(operationPanel, 'Units', 'normalized', ...
    'Position', [0.0, 0.735, 1.0, 0.11], 'Title', 'Recording', 'BackgroundColor', 'w');
recordButton = uicontrol(recordPanel, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.06, 0.46, 0.88, 0.44], 'String', 'Start Recording', ...
    'BackgroundColor', [0.80, 0.95, 0.82], 'FontWeight', 'bold', ...
    'Callback', callbacks.ToggleRecording, 'Tag', 'hairsimRecordButton');
recordStatusText = uicontrol(recordPanel, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.06, 0.04, 0.88, 0.36], 'String', 'Not recording', ...
    'BackgroundColor', 'w', 'FontSize', 8, 'HorizontalAlignment', 'left');

centerPanel = uipanel(operationPanel, 'Units', 'normalized', 'Position', [0.0, 0.0, 1.0, 0.725], ...
    'Title', configValue(config, 'CenterTitle', ''), 'BackgroundColor', 'w');

ui = struct();
ui.controlPanel = controlPanel;
ui.robotSetupPopup = robotSetupPopup;
ui.basePlacementOptions = basePlacementOptions;
ui.statusText = statusText;
ui.recordButton = recordButton;
ui.recordStatusText = recordStatusText;
ui.centerPanel = centerPanel;
if isfield(config, 'HairModelingConfig') && isfield(config, 'HairModelingCallbacks')
    hairModelingPanel = uipanel(controlPanel, 'Units', 'normalized', ...
        'Position', [0.06, 0.03, 0.88, 0.36], 'Title', 'Hair Modeling', 'BackgroundColor', 'w');
    ui.hair = hairsim.hair.createModelingControlPanel( ...
        hairModelingPanel, config.HairModelingConfig, config.HairModelingCallbacks);
end
end

function value = configValue(config, fieldName, defaultValue)
value = defaultValue;
if isfield(config, fieldName)
    value = config.(fieldName);
end
end
