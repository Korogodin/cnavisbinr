classdef CNavisBinr < handle
    %CNavisBinr Parser of NAVIS binary protocol
        
    properties
        Mode_Device = 1;
        Mode_Dump = 2;
        Mode
        
        Signal_GPS = 1;
        Signal_GLO_ST_L1 = 2;
        Signal_GLO_ST_L2 = 3;
        Signal_SBAS = 4;
        Signal_GLO_VT_L1 = 5;
        Signal_GLO_VT_L2 = 6;
        Signal_Gal_E1 = 8;
        Signal_Gal_E5a = 9;
        Signal_Gal_E5b = 10;
        
        IOFileName
        DumpFileName
        Dumpid
        BaudRate
        IOid
        
        Byte
        Data
        DataSize
        PacketNumber
        
        % 88h
        X, Y, Z % rad or m
        Vx, Vy, Vz % m/s
        Solution % [prev 2D reserve diff RAIM diff2 reserve reserve]
        RMS % m
        TimeOfWeek % ms
        NOfWeek
        XO_Shift % ms
        
        % 87h
        SNR_GPS = zeros(1, 32); % shift = 0:  1:32
        SNR_GLO_ST_L1 = zeros(1, 21); % shift = +8 : -7:13
        
        % 60h
        GPS_in_Solution
        GLO_in_Solution
        HDOP
        VDOP
        
    end
    
    methods
        function NB = CNavisBinr()
            NB.DumpFileName = 'dump.bin';
        end
        
        function setMode(NB, Mode)
            NB.Mode = Mode;
        end    
        
        function openDevice(NB, IOFileName, BaudRate)
            NB.BaudRate = BaudRate;
            NB.IOFileName = IOFileName;
            try
                s = serial(NB.IOFileName);
                fopen(s); fclose(s); % Hack for port settings
            end
            system(sprintf('stty -F %s %d', NB.IOFileName, NB.BaudRate));
            NB.IOid = fopen(NB.IOFileName, 'w+');
            NB.Dumpid = fopen(NB.DumpFileName, 'w');
        end
        
        function openDump(NB, IOFileName)
            NB.IOFileName = IOFileName;
            [NB.IOid, mesg] = fopen(NB.IOFileName,'r');
            if NB.IOid == -1
                fprintf('Error of dump-file opening:\n%s\n', mesg);
            end
        end
        
        function ok = getPacketData(NB)
           
            NB.Data = '';
            NB.DataSize = 0;
            NB.PacketNumber = 0;

            Prev_x10 = 0;
            for byte_counter = 1:1024
                if NB.getNextByte
                    if strcmp(NB.Byte, '10') % 10h is special byte
                        if Prev_x10
                            NB.Data = [NB.Data '10'];
                            NB.DataSize = NB.DataSize + 1;
                            Prev_x10 = 0;
                        else
                            Prev_x10 = 1;
                        end
                    else
                        if Prev_x10
                            if strcmp(NB.Byte, '03')
                                if NB.PacketNumber
                                    ok = 1;
                                else
                                    ok = 0;
                                end
                                return;
                            else
                                NB.PacketNumber = NB.Byte;
                            end
                        else
                            NB.Data = [NB.Data NB.Byte];
                            NB.DataSize = NB.DataSize + 1;
                        end
                        Prev_x10 = 0;
                    end
                else
%                     fprintf('Can''t get next byte\n');
                    ok = 0;
                    NB.DataSize = 0;
                    return;
                end
            end
            ok = 0;
        end
        
        function ok = getNextByte(NB)
            if NB.Mode == NB.Mode_Dump
                if ~feof(NB.IOid)
                    NB.Byte = dec2hex(fread(NB.IOid, 1, 'uint8'), 2);
                    ok = 1;
                else
                    NB.Byte = NaN;
                    ok = 0;
                end
            elseif NB.Mode == NB.Mode_Device
                [A, A_count] = fread(NB.IOid, 1, 'uint8');
                if A_count == 1
                    NB.Byte = dec2hex(A, 2);
                    ok = 1;
                    fwrite(NB.Dumpid, A, 'uint8');
                else
                    NB.Byte = NaN;
                    ok = 0;
                end
            end
        end
        
        function ok = parseData(NB)
            switch NB.PacketNumber
                case '60'
                    ok = NB.parse_60h;
                case '87'
                    ok = NB.parse_87h;
                case '88'
                    ok = NB.parse_88h;
                otherwise
                    fprintf('Warning: I do not have a parser for packet %sh:\n%s\n', NB.PacketNumber, NB.Data)
                    ok = 0;
            end
        end
        
        function ok = parse_60h(NB)

            %Number of GPS SV in solution
            shift = 0; % in bytes
            type = 'INT8U';
            hexval = NB.SeparateData(shift, type);
            NB.GPS_in_Solution = NB.ConvertFormat(hexval, type);

            %Number of GLO SV in solution
            shift = 1; % in bytes
            type = 'INT8U';
            hexval = NB.SeparateData(shift, type);
            NB.GLO_in_Solution = NB.ConvertFormat(hexval, type);

            %HDOP
            shift = 2; % in bytes
            type = 'FP32';
            hexval = NB.SeparateData(shift, type);
            NB.HDOP = NB.ConvertFormat(hexval, type);
            
            %VDOP
            shift = 6; % in bytes
            type = 'FP32';
            hexval = NB.SeparateData(shift, type);
            NB.VDOP = NB.ConvertFormat(hexval, type);
            
            ok = 1;
        end
        
        
        function ok = parse_88h(NB)

            %Latitude, rad or X, m
            shift = 0; % in bytes
            type = 'FP64';
            hexval = NB.SeparateData(shift, type);
            NB.X = NB.ConvertFormat(hexval, type);
            
            %Longtitude, rad or Y, m
            shift = 8; % in bytes
            type = 'FP64';
            hexval = NB.SeparateData(shift, type);
            NB.Y = NB.ConvertFormat(hexval, type);
            
            %Height, m or Z, m
            shift = 16; % in bytes
            type = 'FP64';
            hexval = NB.SeparateData(shift, type);
            NB.Z = NB.ConvertFormat(hexval, type);       
            
            %Plane RMS, m 
            shift = 24; % in bytes
            type = 'FP32';
            hexval = NB.SeparateData(shift, type);
            NB.RMS = NB.ConvertFormat(hexval, type);        
            
            %TimeOfWeek, ms 
            shift = 28; % in bytes
            type = 'FP80';
            hexval = NB.SeparateData(shift, type);
            NB.TimeOfWeek = NB.ConvertFormat(hexval, type);              

            %TimeOfWeek, ms 
            shift = 38; % in bytes
            type = 'INT16S';
            hexval = NB.SeparateData(shift, type);
            NB.NOfWeek = NB.ConvertFormat(hexval, type);      
            
            %Vx, m/s  (X of lat)
            shift = 40; % in bytes
            type = 'FP64';
            hexval = NB.SeparateData(shift, type);
            NB.Vx = NB.ConvertFormat(hexval, type);                

            %Vy, m/s  (Y of long)
            shift = 48; % in bytes
            type = 'FP64';
            hexval = NB.SeparateData(shift, type);
            NB.Vy = NB.ConvertFormat(hexval, type);                
            
            %Vz, m/s  (Z of lat)
            shift = 56; % in bytes
            type = 'FP64';
            hexval = NB.SeparateData(shift, type);
            NB.Vz = NB.ConvertFormat(hexval, type);                
            
            %Oscillator shift, ms  
            shift = 64; % in bytes
            type = 'FP32';
            hexval = NB.SeparateData(shift, type);
            NB.XO_Shift = NB.ConvertFormat(hexval, type);                
            
            %Solution, flags
            shift = 68; % in bytes
            type = 'INT8U';
            hexval = NB.SeparateData(shift, type);
            NB.Solution = NB.ConvertFormat(hexval, type);                
            
            ok = 1;
        end
        
        function ok = parse_87h(NB)
            
            Nsat = length(NB.Data) / 2 / 20;
            
            if Nsat ~= round(Nsat)
                ok = 0;
                return;
            end

            NB.SNR_GPS = NB.SNR_GPS*0;
            NB.SNR_GLO_ST_L1 = NB.SNR_GLO_ST_L1*0;
            for i = 1:Nsat
                
            %Signal 
            shift = (i-1)*20 + 0; % in bytes
            type = 'INT8U';
            hexval = NB.SeparateData(shift, type);
            Sign = NB.ConvertFormat(hexval, type); 
            
            shift = (i-1)*20 + 1; % in bytes
            type = 'INT8S';
            hexval = NB.SeparateData(shift, type);
            SV_Num = NB.ConvertFormat(hexval, type); 
            
            shift = (i-1)*20 + 2; % in bytes
            type = 'INT8U';
            hexval = NB.SeparateData(shift, type);
            Sign_SNR = NB.ConvertFormat(hexval, type); 
            
            switch Sign
                case NB.Signal_GPS
                    if (SV_Num >= 1)&&(SV_Num <= 32)
                        NB.SNR_GPS(SV_Num) = Sign_SNR;
                    end
                case NB.Signal_GLO_ST_L1
                    if (SV_Num >= -7)&&(SV_Num <= 13)
                        NB.SNR_GLO_ST_L1(SV_Num+8) = Sign_SNR;
                    end
            end
            end
            
            ok = 1;
        end
        
        function hexval = SeparateData(NB, shift, type)
            
            shifthex = shift * 2; % in half-bytes
            switch type
                case 'INT8U'
                    strend = shifthex + 2; % 2*2 
                case 'INT8S'
                    strend = shifthex + 2; % 2*2 
                case 'INT16S'
                    strend = shifthex + 4; % 2*2 
                case 'FP32'
                    strend = shifthex + 8; % 4*2
                case 'FP64'
                    strend = shifthex + 16; % 8*2
                case 'FP80'
                    strend = shifthex + 20; % 10*2 
            end
            hexval = NB.Data(shifthex+1:strend);
        end
        
        function v = ConvertFormat(NB, hexval, type)
            % See Appendix A.1
            switch type
                case 'INT8U'
                    v = hex2dec(hexval);
                    
                case 'INT8S'
                    binval = dec2bin(hex2dec(hexval(1:2)), 8);
                    s = bin2dec(binval(1));
                    f = bin2dec(binval(2:8));
                    v = (-1)^s * f;                    
                    
                case 'INT16S'
                    % Reverse bytes in word
                    binval = dec2bin(hex2dec([hexval(3:4) hexval(1:2)]), 16);

                    s = bin2dec(binval(1));
                    f = bin2dec(binval(2:16));
                    
                    v = (-1)^s * f;
                                    
                case 'FP32'
                    % Reverse bytes in word
                    binval = dec2bin(hex2dec([hexval(7:8) hexval(5:6) hexval(3:4) hexval(1:2)]), 32);

                    s = bin2dec(binval(1));
                    e = bin2dec(binval(2:9));
                    f = bin2dec(binval(10:32));
                    
                    if (0 < e) && (e < 255)
                        v = (-1)^s * 2^(e-127) * (1 + f/ (2^23));
                    elseif (e == 0) && (f ~= 0)
                        v = (-1)^s * 2^(-126) * f / (2^23);
                    elseif (e == 0) && (f == 0)
                        v = (-1)^s * 0;
                    elseif (e == 2047) && (f == 0)
                        v = (-1)^s * inf;
                    elseif (e == 2047) && (f ~= 0)
                        v = NaN;
                    end
                    
                case 'FP64'
                    % Reverse bytes in word
                    binval = dec2bin(hex2dec([hexval(15:16) hexval(13:14) hexval(11:12) hexval(9:10) hexval(7:8) ...
                        hexval(5:6) hexval(3:4) hexval(1:2)]), 64);

                    s = bin2dec(binval(1));
                    e = bin2dec(binval(2:12));
                    f = bin2dec(binval(13:64));
                    
                    if (0 < e) && (e < 2047)
                        v = (-1)^s * 2^(e-1023) * (1 + f/ (2^52));
                    elseif (e == 0) && (f ~= 0)
                        v = (-1)^s * 2^(-1022) * f / (2^52);
                    elseif (e == 0) && (f == 0)
                        v = (-1)^s * 0;
                    elseif (e == 2047) && (f == 0)
                        v = (-1)^s * inf;
                    elseif (e == 2047) && (f ~= 0)
                        v = NaN;
                    end
                    
                case 'FP80'
                    % Reverse bytes in word
                    binval = dec2bin(hex2dec([hexval(19:20) hexval(17:18) hexval(15:16) hexval(13:14) ...
                        hexval(11:12) hexval(9:10) hexval(7:8) hexval(5:6) hexval(3:4) hexval(1:2)]), 80);

                    s = bin2dec(binval(1));
                    e = bin2dec(binval(2:16));
                    i = bin2dec(binval(17));
%                     f = bin2dec(binval(18:80));
                    f1 = bin2dec(binval(18:60));
                    f2 = bin2dec(binval(61:80));
                    f = f1*2^20 + f2;
                    
                    if (0 < e) && (e < 32767)
                        v = (-1)^s * 2^(e-16383) * (1 + f/ (2^63));
                    elseif (e == 0) && (f ~= 0)
                        v = (-1)^s * 2^(-16382) * f / (2^63);
                    elseif (e == 0) && (f == 0)
                        v = (-1)^s * 0;
                    elseif (e == 2047) && (f == 0)
                        v = (-1)^s * inf;
                    elseif (e == 2047) && (f ~= 0)
                        v = NaN;
                    end
                otherwise
                    fprintf('Unknown format in functionConvertFormat\n');
            end
        end
        
         function request_88h_bymeansof_27h(NB, Period)
            NB.sendPacket('27', dec2hex(Period, 2));
         end
         
         function request_87h_bymeansof_39h(NB, Period)
            NB.sendPacket('39', dec2hex(Period, 2));
         end         
        
         function request_60h_bymeansof_21h(NB, Period)
            NB.sendPacket('21', dec2hex(Period, 2));
            pause(0.25);
            NB.sendPacket('21', dec2hex(Period, 2));
         end
         
         function sendPacket(NB, PacketNumber, Data)
             if NB.Mode == NB.Mode_Device
                fwrite(NB.IOid, hex2dec('10'), 'uint8');
                fwrite(NB.IOid, hex2dec(PacketNumber), 'uint8');
                for j = 1:(length(Data)/2)
                    fwrite(NB.IOid, hex2dec(Data((2*j-1):(2*j))), 'uint8');
                end
                fwrite(NB.IOid, hex2dec('10'), 'uint8');
                fwrite(NB.IOid, hex2dec('03'), 'uint8');
             else
                 fprintf('This function is available in Mode_Device only\n');
             end
         end
         
         function reset_woerase(NB)
             NB.sendPacket('01', '000121010001');
             pause(2);
             NB.sendPacket('01', '000121010001');
         end
         
         function reset_erase(NB)
             NB.sendPacket('01', '000121010001');
             pause(2);
             NB.sendPacket('01', '000121010000');
             pause(2);
             NB.sendPacket('01', '000121010000');
         end

        
    end
    
end

