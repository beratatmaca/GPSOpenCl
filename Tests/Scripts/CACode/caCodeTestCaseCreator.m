clc;
clear;
addpath('../MatlabFunctions');

testCaseName = {'CaCodeTestCase1', 'CaCodeTestCase2', 'CaCodeTestCase3', 'CaCodeTestCase4'};
Fs = [2.048e6, 4.096e6, 8.192e6, 16.384e6];


for iter = 1 : length(Fs)
    settings.samplingFreq       = Fs(iter);
    settings.codeFreqBasis      = 1.023e6;
    settings.codeLength         = 1023;

    caCodesTable = makeCaTable(settings);
    
    fileName = char(strcat(testCaseName(iter), '.txt'));
    fileInputID = fopen(fileName,'w');
    [prns , lengthOfCode] = size(caCodesTable);
    for i = 1: prns
        for j = 1:lengthOfCode
            fprintf(fileInputID, '%f\n', caCodesTable(i,j));
        end
    end    
    fclose(fileInputID);
end