clc;
clear;
addpath('../MatlabFunctions');

testCaseName = {'AcquisitionDopplerSignals', 'CorrelationResults', 'AcquisitionMetrics'};


SVs = 1:32;
settings.samplingFreq       = 4.096e6;
settings.codeFreqBasis      = 1.023e6;
settings.codeLength         = 1023;
settings.acqSearchBand      = 8;
settings.IF                 = 0;

samplesPerCode = round(settings.samplingFreq / ...
                        (settings.codeFreqBasis / settings.codeLength));
                    
fid = fopen('inputSignal.txt', 'r');
data = fscanf(fid, '%f\n', samplesPerCode * 2)';
data1=data(1:2:end);
data2=data(2:2:end);
data=data1 + 1i .* data2;
                    
ts = 1 / settings.samplingFreq;
phasePoints = (0 : (samplesPerCode-1)) * 2 * pi * ts;
numberOfFrqBins = round(settings.acqSearchBand * 2) + 1;

caCodesTable = makeCaTable(settings);
frqBins = zeros(1, numberOfFrqBins);
results = zeros(length(SVs), numberOfFrqBins, samplesPerCode);

%% Doppler

fileNameDoppler = char(strcat(testCaseName(1), '.txt'));
fileInputIDDoppler = fopen(fileNameDoppler,'w');
for frqBinIndex = 1:numberOfFrqBins
    frqBins(frqBinIndex) = settings.IF - ...
                           (settings.acqSearchBand/2) * 1000 + ...
                           0.5e3 * (frqBinIndex - 1);
    sigCarr = exp(1i*frqBins(frqBinIndex) * phasePoints);
    
    for i = 1: length(sigCarr)
        fprintf(fileInputIDDoppler, '%f\n', real(sigCarr(1,i)));
        fprintf(fileInputIDDoppler, '%f\n', imag(sigCarr(1,i)));
    end    
end 
fclose(fileInputIDDoppler);

%% Correlation
sv = 1;
caCodeFreqDom = conj(fft(caCodesTable(sv, :)));    
for frqBinIndex = 1:numberOfFrqBins
    frqBins(frqBinIndex) = settings.IF - ...
                           (settings.acqSearchBand/2) * 1000 + ...
                           0.5e3 * (frqBinIndex - 1);
    sigCarr = exp(1i*frqBins(frqBinIndex) * phasePoints);
    I1      = real(sigCarr .* data);
    Q1      = imag(sigCarr .* data);
    IQfreqDom1 = fft(I1 + 1i*Q1);
    convCodeIQ1 = IQfreqDom1 .* caCodeFreqDom;
    acqRes1 = abs(ifft(convCodeIQ1)) .^ 2;
    results(sv, frqBinIndex, :) = acqRes1;
end

fileNameCorrelation = char(strcat(testCaseName(2), '.txt'));
fileInputIDCorrelation = fopen(fileNameCorrelation,'w');
[numOfSatellites, numberOfFreqs, numOfCodes] = size(results);

for i = 1: numberOfFreqs
    for j = 1: numOfCodes
        fprintf(fileInputIDCorrelation, '%f\n', results(1, i, j));
    end
end    
fclose(fileInputIDCorrelation);

%% Find Acquisition Metrics
clear results;
for sv = 1:32
    caCodeFreqDom = conj(fft(caCodesTable(sv, :)));    
    for frqBinIndex = 1:numberOfFrqBins
        frqBins(frqBinIndex) = settings.IF - ...
                               (settings.acqSearchBand/2) * 1000 + ...
                               0.5e3 * (frqBinIndex - 1);
        sigCarr = exp(1i*frqBins(frqBinIndex) * phasePoints);
        I1      = real(sigCarr .* data);
        Q1      = imag(sigCarr .* data);
        IQfreqDom1 = fft(I1 + 1i*Q1);
        convCodeIQ1 = IQfreqDom1 .* caCodeFreqDom;
        acqRes1 = abs(ifft(convCodeIQ1)) .^ 2;
        results(frqBinIndex, :) = acqRes1;
    end
    [~, frequencyBinIndex] = max(max(results, [], 2));
    [peakSize, codePhase] = max(max(results));
    meanValue = mean(mean(results));

    acqResults.peakValue(sv) = peakSize;
    acqResults.codePhase(sv) = codePhase;
    acqResults.dopplerFrequency(sv) = frqBins(frequencyBinIndex);
    acqResults.noiseLevel(sv) = meanValue;
    acqResults.cno(sv) = 10*log10((peakSize/meanValue)/1e-3);
end

fileName = char(strcat(testCaseName(3), '.txt'));
fileInputID = fopen(fileName,'w');
for sv = 1:32
    fprintf(fileInputID, '%d\n', sv);
    fprintf(fileInputID, '%f\n', acqResults.peakValue(sv));
    fprintf(fileInputID, '%f\n', acqResults.codePhase(sv));
    fprintf(fileInputID, '%f\n', acqResults.dopplerFrequency(sv));
    fprintf(fileInputID, '%f\n', acqResults.noiseLevel(sv));
    fprintf(fileInputID, '%f\n', acqResults.cno(sv));
end
fclose(fileInputID);
