clc;
clear;

testCaseName = {'FFTTestCase1', 'FFTTestCase2', 'FFTTestCase3', 'FFTTestCase4', ...
                'IFFTTestCase1', 'IFFTTestCase2', 'IFFTTestCase3', 'IFFTTestCase4'};
Fs = [2.048e6, 4.096e6, 8.192e6, 16.384e6, 2.048e6, 4.096e6, 8.192e6, 16.384e6];
f = [5e5, 2e5, 2.1e6, 4.9e6, 5e5, 2e5, 2.1e6, 4.9e6];
direction = [1, 1, 1, 1, -1, -1, -1, -1];

for iter = 1 : length(Fs)

    [out, in] = fftTestCaseGenerate(Fs(iter), f(iter), direction(iter));
    
    fileNameInput = char(strcat(testCaseName(iter), 'Input.txt'));
    fileInputID = fopen(fileNameInput,'w');
    for dataIter = 1: length(in)
        fprintf(fileInputID, '%f\n', real(in(dataIter)));
        fprintf(fileInputID, '%f\n', imag(in(dataIter)));
    end    
    fclose(fileInputID);
    
    fileNameOutput = char(strcat(testCaseName(iter), 'Output.txt'));
    fileOutputID = fopen(fileNameOutput,'w');
    for dataIter = 1: length(in)
        fprintf(fileOutputID, '%f\n', real(out(dataIter)));
        fprintf(fileOutputID, '%f\n', imag(out(dataIter)));
    end    
    fclose(fileOutputID);
end

function [output, input] = fftTestCaseGenerate(Fs, f, dir)
t = 0 : 1/Fs : (1e-3 - 1/Fs);
input = zeros(1, length(t));

for iter = 1:length(t)
    input(1, iter) = exp(2 * pi * 1i * f * t(iter)) + randn();
end

if dir == 1
    output = fft(input);
else
    output = ifft(input);
end

end
