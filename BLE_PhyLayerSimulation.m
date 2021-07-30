%% BLE PHY Layer
%% Wave Generation 
pathLossModel = 'Free space';  % Path loss model
phyMode = 'LE1M';     % PHY transmission mode (uncoded 1Mbps)
rxSensitivity = -70 ; % Receiver sensitivity in dBm(upon the PHY mode)
txPower = 0;          % Transmit power in dBm(txpower=1mW)
txAntennaGain = 0;    % Transmitter antenna gain in dB
rxAntennaGain = 0;    % Receiver antenna gain in dB
linkMargin = 15;      % Link margin(dB)(assumed)- Link margin = Min. received power - Receiver sensitivity

samplesPerSymbol = 8; % Samples per symbol
dataLen = 254;        % Data length in bytes

% Advertising channel index (used by data-whitening block to randomize the bits)
chanIndex =37;
% Center frequency in Hz based on the channel index
fc = 2402*1e6;
% Default access address for periodic advertising channels(Channel index >=37)
accessAdd = [0 1 1 0 1 0 1 1 0 1 1 1 1 1 0 1 1 0 0 1 0 0 0 1 0 1 1 1 0 0 0 1]';

% Random data bits generation
txBits = randi([0 1],dataLen*8,1,'int8');

% Generate BLE waveform
txWaveform = bleWaveformGenerator(txBits,'Mode',phyMode,...
                    'SamplesPerSymbol',samplesPerSymbol,...
                    'ChannelIndex',chanIndex,...
                    'AccessAddress',accessAdd);


%% RF Impairments
% Create and configure the System objects for impairments
initImp = helperBLEImpairmentsInit(phyMode,samplesPerSymbol);

% Configure RF impairments
initImp.dc = 20;                    % Percentage related to maximum amplitude value
initImp.pfo.FrequencyOffset = 5800; % Frequency offset in Hz
initImp.pfo.PhaseOffset = 5;        % Phase offset in degrees
initoff = 0.15*samplesPerSymbol;    % Static timing offset
stepsize = 20*1e-6;                 % Timing drift in ppm, Max range is +/- 50 ppm
initImp.vdelay = (initoff:stepsize:initoff+stepsize*(length(txWaveform)-1))';


% Pass generated BLE waveform through RF impairments
txImpairedWfm = helperBLEImpairmentsAddition(txWaveform,initImp);

%% SNR
NF = 6;            % Noise figure(dB)(NF=10log(Te/To))(Safety margin 6dB)
T = 290;           % Ambient temperature (K)
dBm2dBFactor = 30; % Factor for converting dBm to dB

% Symbol rate based on the PHY transmission mode
symbolRate = 1e6; 
BW = samplesPerSymbol*symbolRate; % Bandwidth (Hz)
k = 1.3806e-23;                   % Boltzmann constant (J/K)
noiseFloor = 10*log10(k*T*BW)+NF; % Nosie floor in dB - Noise floor includes thermal noise & atmospheric noise

% Measure signal power at the receiver based on the receiver sensitivity and assumed link margin
measuredPowerVector = (rxSensitivity - dBm2dBFactor)+linkMargin;% Minimum recieved power(assumed)
snrdB = measuredPowerVector - noiseFloor; % SNR in dB

%% Pathloss
% Obtain the path loss value in dB
pldB = txPower-dBm2dBFactor+rxAntennaGain+txAntennaGain-measuredPowerVector;% pldB=P(transmitted)-P(recieved)
plLinear = 10^(pldB/20); % Convert from dB to linear scale

% Attenuate BLE waveform
attenWaveform  = txImpairedWfm./plLinear;

%% Gausian noise
% Add white Gaussian noise to the attenuated BLE waveform
rxWaveform = awgn(attenWaveform,snrdB,'measured'); % white noise is a random signal having equal intensity at different frequencies

%% Reciever
% Create and configure the receiver System objects 
initRxParams = helperBLEReceiverInit(phyMode,samplesPerSymbol,accessAdd);

% Recover data bits using practical receiver
[rxBits,accessAddress] = helperBLEPracticalReceiver(rxWaveform,initRxParams,chanIndex);

%% BER & Distance
% Obtain BER by comparing the transmitted and recieved bits
ber = [];
if(length(txBits) == length(rxBits))
    ber = (sum(xor(txBits,rxBits))/length(txBits));
end
% Distance between transmitter and reciever
distance = helperBluetoothEstimateDistance(pathLossModel,pldB,fc);

%% Plots
disp(['Estimated outputs: ', newline , '    Path loss : ', num2str(pldB), ' dB', newline, ...
    '    SNR : ', num2str(snrdB), ' dB', newline ,'    BER: ', num2str(ber), newline ,...
    '    Distance: ', num2str(distance), ' m']);

% Plot the spectrum of the transmitted and received BLE waveform
specAnalyzer = dsp.SpectrumAnalyzer('NumInputPorts',2,'SampleRate',symbolRate*samplesPerSymbol,...
    'Title','Spectrum of Transmitted and Received BLE Signals',...
   'ShowLegend',true,'ChannelNames',{'Transmitted BLE signal','Received BLE signal'});
specAnalyzer(txWaveform,rxWaveform);
release(specAnalyzer);

% Plot the spectrum of After-Noise BLE waveforms
specAnalyzer2 = dsp.SpectrumAnalyzer('NumInputPorts',2,'SampleRate',symbolRate*samplesPerSymbol,...
    'Title','Spectrum of After-Noise BLE Signals',...
   'ShowLegend',true,'ChannelNames',{'Impairmented signal','PL-Signal'});
specAnalyzer2(txImpairedWfm,attenWaveform);
release(specAnalyzer2);


% Plot Tx & Rx bits
figure 
subplot(211)
plot(txBits)
title('Transmitted bits')
subplot(212)
plot(rxBits)
title('Recieved bits')
