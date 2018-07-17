clear; clc; close all

%% LPF cutoff frequency
codes_lpf = [0, bin2dec('0010000'), bin2dec('0010001'), bin2dec('0011001'), bin2dec('0101011'), bin2dec('1000001'), bin2dec('1011001'), bin2dec('1011010'), bin2dec('1111111')];
cutoff_frequency = [9.47, 13.54, 13.79, 15.78, 20.06, 25.05, 30.06, 30.26, 36.95];
EstimatePolyfit(codes_lpf, cutoff_frequency, 0:1:127, "Cutoff frequency vs codes", "Code, dimensionless", "Cutoff frequency, MHz");
EstimatePolyfit(cutoff_frequency, codes_lpf, 9.47:0.01:36.95, "Codes vs cutoff frequency", "Cutoff frequency, MHz", "Code, dimensionless");

%% IFA gain
codes_gain = [bin2dec('0000000000'), bin2dec('0100001110'), bin2dec('0100101100'), bin2dec('0101001010'), bin2dec('0110000110'), bin2dec('0110100100'), bin2dec('0111000010'), bin2dec('0111111110'), bin2dec('1000011100'), bin2dec('1001011000'), bin2dec('1010010100'), bin2dec('1011010000'), bin2dec('1011101110'), bin2dec('1100001100'), bin2dec('1100101010'), bin2dec('1111000000'), bin2dec('1111011110')];
gain = [1.12, 1.30, 4.55, 10.00, 12.98, 18.82, 23.31, 26.10, 32.03, 37.79, 44.38, 46.52, 52.13, 54.62, 63.04, 67.25, 67.30];
EstimatePolyfit(codes_gain, gain, 0:1:1023, "IFA gain vs codes", "Code, dimensionless", "IFA gain, dB");
%EstimatePolyfit(gain, codes_gain, 1:0.1:67.30, "Codes vs IFA gain", "IFA gain, dB", "Code, dimensionless");

%% IFA percents
codes_percents = [bin2dec('0000000000'),bin2dec('0101000000'),bin2dec('0111111111'),bin2dec('1100110010'),bin2dec('1111111111')];
percets = [0,30,45,73,96];
EstimatePolyfit(codes_percents, percets, 0:1:1023, "IFA percents vs codes", "Code, dimensionless", "IFA percents");

%% IQ offset
codes_offset = [bin2dec('000000'),bin2dec('011111'),bin2dec('100000'),bin2dec('100001'),bin2dec('111111')];
offset = [74.1, 89.8, 90, 90.3, 104.9];
EstimatePolyfit(codes_offset, offset, 0:1:bin2dec('111111'), "IQ offset vs codes", "Code, dimensionless", "IQ offset");
dst = EstimatePolyfit(offset, codes_offset, 74.1:0.01:104.9, "Codes vs IQ offset", "IQ offset", "Code, dimensionless");
