%This function get one or two input arguments.
%   If variable argument in == 1 -> Read all sample of data base.
%        The input argument must be the name of data base.
%   If variable arguments in ==2 -> Read N sample of data base
%       The first input argument must be the name of data base.
%       The second argument must be the number of samples.

% signal
%        NxM matrix (doubles) of M signals with each signal being N samples long.
%        Signal dataype will be either in double int16 format
%        depending on the flag passed to the function (according to
%        the boolean flags below).
%  tm
%        Nx1 vector of doubles representing the sampling intervals
%        (elapsed time in seconds).
%  Fs    (Optional)
%        1x1 Double, sampling frequency in Hz of the first signal in signalList
%        (default =1).

function [tm,ecg,fs]=ECG_DATA(str)
str
%if (narargin==1)
    %Read all sample ECG signal from MIT-BIH Arrhythmia Database
    [tm,ecg,fs]=rdsamp(str,1);
%elseif (narargin==2)
    %Read N sample ECG signal from MIT-BIH Arrhythmia Database
%    [tm,ecg,fs]=rdsamp(str,1,N);
end