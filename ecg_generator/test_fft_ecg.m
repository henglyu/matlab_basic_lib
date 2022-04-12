function test_fft_ecg(time,ecg)

%[time,ecg]=ECG_DATA;

FT=abs(fft(ecg));


length(FT)
x=[];
for i=1:length(FT);
    if (FT(i)>10*mean(FT))
        x=horzcat(x,i);
    end
end

subplot(211)
plot(time,ecg,'linewidth',2);
subplot(212)
stem(FT);
end
    