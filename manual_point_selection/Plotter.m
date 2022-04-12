clear
close all
clc

%% Laoding Data

load('sampleData/sampleECG.mat');          % Loading Raw Data and Sampling Freq

Signal = ECG;
ts = 1/fs;                      % Sampling Time
SigLen = length(Signal);
TimeLen = SigLen/fs;

t = linspace(0, TimeLen, SigLen); % Time axis Creation

%% Create Plots to Selecte R-peaks Manually

Counter = 0;                    % Index Counter

%Data Seperator
TimeSlice = 10;                 % Plot Time Slice in Seconds
DataNum = TimeSlice * fs;       % Nummber of Data in each Slice

PlotNum = ceil(TimeLen/TimeSlice);     % Number of Plots to Create

Min_Ind = 0;                    % Store Last Index of the Previous Plot
Index = [];
for iter = 1:PlotNum

    Break_Flag = 0;         % Loop Break Flag

    % Check if its the last Iteration last R-Peak Index is the Last Signal Index
    if (iter == PlotNum)
        Ind = (1+(iter-1) * DataNum : numel(Signal));
    else
        Ind = (1+(iter-1) * DataNum : iter*DataNum+30);   % Seperation Index Creation
    end

    Min_Ind = Ind(1)-1;

    % Do the Current Plot
    plot(t(Ind), Signal(Ind), 'Marker', 'none')
    title(['Iteration = ' num2str(iter) '    Interval Start = ' num2str((iter-1)...
        *TimeSlice) '     Interval Stop = ' num2str(iter * TimeSlice)])

    while true

        dcm_obj = datacursormode(gcf);      % Create Current Data Cursor Object
        set(dcm_obj, 'DisplayStyle', 'window', 'SnapToDataVertex', 'on', 'Enable', 'on')

        w = waitforbuttonpress;             % Wait for Button Press!!
        if (w == 1)                         % if Pressed Up Coming Button is from KeyBoard

            Current_Key = get(gcf, 'CurrentCharacter');      % Get Current Pressed Key

            if (Current_Key == 'c')         % Close if Current Key is 'c' for Closed
                disp('     Closing The Program!....')
                Break_Flag = 1;             % Activate to Close Completely
                break

            elseif (Current_Key == 'n')     % Go to Next Plot if Current Key is 'n' for Next

                if (iter == PlotNum)

                    close all
                    fprintf("\n\nAll Done!\n\n")

                else
                    % Current Plot Info
                    disp('Next Plot...')
                    disp(' =========================================================== ')
                    disp(['Iteration = ' num2str(iter) '    Interval Start = ' ...
                        num2str((iter-1) * TimeSlice) '     Interval Stop = '...
                        num2str(iter * TimeSlice)])
                    disp(' =========================================================== ')
                    break
                end
            end

        elseif (w == 0)                    % if Pressed Button is from Mouse

            info_struct = getCursorInfo(dcm_obj);       % get Cursor Info

            % If it is not Struct then it is the Click was not on the Plot
            % So We Check
            if (class(info_struct) == 'struct')
                if (Counter == 0)
                    Counter = 1;
                    Index = info_struct.DataIndex;
                    disp(['Index  1 Captured!'])
                else
                    if (info_struct.DataIndex + Min_Ind - Index(end) > 30)
                        Counter = Counter + 1;
                        Index = [Index; info_struct.DataIndex + Min_Ind];
                        disp(['Index  ' num2str(Counter) '  Captured!'])
                    end
                end
            end
        end
    end

    % If 'c' is Pressed then Break!
    if (Break_Flag == 1)
        close all
        break
    end
end

%% Saving the index and the time
Index = sort(unique(Index));
SelectedIndex = Index;
IndTime = t(Index);

save IndexSelected.mat SelectedIndex IndTime

%% Plot Results
semilogy(t, Signal, 'b-')
hold on
semilogy(IndTime, Signal(SelectedIndex), 'ro', 'LineWidth', 5)
title('Whole Data with Selected Indices.')
xlabel('Time')
ylabel('uV')

clear