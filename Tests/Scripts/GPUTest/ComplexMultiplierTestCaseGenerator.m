clc;
clear;

testCaseName = {'ComplexMultiplierTestCase1', 'ComplexMultiplierTestCase2'};
Fs = [4.096e6, 8.192e6];

for iter = 1 : length(Fs)

    [out, in1, in2] = complexMultiplierTestCaseGenerate(Fs(iter));
    
    fileNameInput = char(strcat(testCaseName(iter), 'Input1.txt'));
    fileInputID = fopen(fileNameInput,'w');
    for dataIter = 1: length(in1)
        fprintf(fileInputID, '%f\n', real(in1(dataIter)));
        fprintf(fileInputID, '%f\n', imag(in1(dataIter)));
    end    
    fclose(fileInputID);
    
    
    fileNameInput = char(strcat(testCaseName(iter), 'Input2.txt'));
    fileInputID = fopen(fileNameInput,'w');
    for dataIter = 1: length(in2)
        fprintf(fileInputID, '%f\n', real(in2(dataIter)));
        fprintf(fileInputID, '%f\n', imag(in2(dataIter)));
    end    
    fclose(fileInputID);
    
    fileNameOutput = char(strcat(testCaseName(iter), 'Output.txt'));
    fileOutputID = fopen(fileNameOutput,'w');
    for dataIter = 1: length(out)
        fprintf(fileOutputID, '%f\n', real(out(dataIter)));
        fprintf(fileOutputID, '%f\n', imag(out(dataIter)));
    end    
    fclose(fileOutputID);
end

function [output, input1, input2] = complexMultiplierTestCaseGenerate(Fs)
t = 0 : 1/Fs : (1e-3 - 1/Fs);
input1 = zeros(1, length(t));
input2 = zeros(1, length(t));

for iter = 1:length(t)
    input1(1, iter) = 1 * (randn() + 1i * randn());
    input2(1, iter) = 1 * (randn() + 1i * randn());
end

output = input1 .* input2;
end
