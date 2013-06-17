clear all
clc
close all

NB = CNavisBinr();
NB.setMode(NB.Mode_Dump);
NB.openDump('test.bin');

ok = 1;
k = 1;
SNR_GPS = NB.SNR_GPS;
SNR_GPS_old = NB.SNR_GPS;
SNR_GPS_oldold = NB.SNR_GPS;
while ok
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
            
            k = k + 1;
            figure(1); subplot(2,1,1); plot(Y, X);
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
    end
    
end
    
    