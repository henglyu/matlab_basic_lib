%This function load ecg data from physionet using WFTD toolbox for Matlab
%from physionet.com.

%   "str" is the name of the MIT's data base to load.
function [tm,ecg,fs]=ECG_Load(str)
h=waitbar(0.5,'50% loading...');
close(h);
N=10000;
%[tm,ecg,fs]=rdsamp('ltstdb/s20011',1,N);
[tm,ecg,fs]=rdsamp(str,1,N);

h=waitbar(0.9,'90% loading...');
pause(2);
close(h);
end
