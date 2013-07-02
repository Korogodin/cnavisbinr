clear
clc
close all

try
    fclose(instrfind);
end

addpath('../OrbitConverter');

NB = CNavisBinr();
NB.setMode(NB.Mode_Device);
NB.openDevice('/dev/ttyUSB1', 38400);
NB.request_88h_bymeansof_27h(1);
pause(0.5);
NB.request_87h_bymeansof_39h(1);
pause(0.5);
NB.sendPacket('B2', [dec2hex(bin2dec('00001000'), 2) dec2hex(bin2dec('00000000'), 2)]); % ECEF
pause(0.5);

SNR_GPS = NB.SNR_GPS;
SNR_GPS_old = NB.SNR_GPS;
SNR_GPS_oldold = NB.SNR_GPS;
k = 1;
while 1
    ok = NB.getPacketData;
    if ok
        NB.parseData;
        if strcmp(NB.PacketNumber, '88')
            X(k) = NB.X;
            Y(k) = NB.Y; 
            Z(k) = NB.Z;
            
            Vx(k) = NB.Vx;
            Vy(k) = NB.Vy;
            Vz(k) = NB.Vz;
            fprintf('Solution RMS = %f m\n', NB.RMS);
            RMS(k) = NB.RMS;
            Solution(k) = NB.Solution;
            TimeofWeek(k) = NB.TimeOfWeek;
            k = k + 1;
            figure(1); subplot(2,1,1); plot3(X, Y, Z);
        end
        if strcmp(NB.PacketNumber, '87')
            SNR_GPS_oldold = SNR_GPS_old;
            SNR_GPS_old = SNR_GPS;
            SNR_GPS = NB.SNR_GPS;
            
            figure(1); subplot(2,1,2); bar(SNR_GPS)
            
            if sum((SNR_GPS - SNR_GPS_oldold) < -2) > 3
                disp('Wall Jammer!')
            end
        end
    else
%         fprintf('No data\n');
        pause(0.4);
    end
end


fclose(NB.IOid);



% ok = 1;
% k = 0;
% while ok
%     ok = NB.getPacketData;
%     if ok; NB.parseData; end
%     k = k+1;
%     X(k) = NB.X;
% end
    
    