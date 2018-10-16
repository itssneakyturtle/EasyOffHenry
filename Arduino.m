function Arduino()
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
% irFlag = 0;
a = arduino();
volts = readVoltage(a,'A0')*0.0048828125;
Distance = 27.86*(volts.^-1)/100
pause(0.01)
if Distance <= 35
%     irFlag = 1
    while Distance <= 35
        pause(0.1);
        volts = readVoltage(a,'A0')*0.0048828125;
        Distance = 27.86*(volts.^-1)/100
    end
%     irFlag = 0;
end
end

