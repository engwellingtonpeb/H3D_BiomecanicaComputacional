function ref = healthyGaitReference()
%HEALTHYGAITREFERENCE Cinematica sagital normativa de marcha saudavel.
%   Curvas aproximadas (Winter 2009 / Perry 1992), velocidade ~1.2 m/s,
%   ciclo ~1.1 s. Fase 0 = contato inicial do membro em questao.
%   Convencoes OpenSim (gait2354): hip_flexion + = flexao,
%   knee_angle - = flexao, ankle_angle + = dorsiflexao. Saida em rad.
%
%   ref.eval(phase) -> struct com q e qd (por unidade de fase) de
%   hip, knee, ankle e hip_add.

% [ % do ciclo , angulo (graus) ]
hip = [0 30; 10 25; 20 15; 30 5; 40 -4; 50 -10; 55 -9; 60 -3; 70 12; 80 26; 87 32; 94 31; 100 30];
kneeFlex = [0 4; 5 11; 15 18; 25 13; 35 6; 42 4; 50 8; 56 20; 62 38; 68 55; 73 60; 80 53; 87 32; 94 10; 100 4];
ankle = [0 0; 5 -5; 10 -3; 20 3; 30 7; 45 10; 50 7; 55 -3; 61 -16; 65 -14; 72 -5; 80 1; 90 1; 100 0];
hipAdd = [0 0; 10 4; 20 5; 40 2; 55 -2; 65 -5; 80 -3; 95 -1; 100 0];

N = 1000;
ph = (0:N-1)'/N;
ref.phase = ph;
ref.hip    = deg2rad(periodicFit(hip, ph));
ref.knee   = -deg2rad(periodicFit(kneeFlex, ph));   % convencao OpenSim
ref.ankle  = deg2rad(periodicFit(ankle, ph));
ref.hipAdd = deg2rad(periodicFit(hipAdd, ph));
ref.N = N;
ref.cycleTime = 1.1;     % s
ref.speed = 1.2;         % m/s
end

function y = periodicFit(kp, ph)
% Interpolacao pchip periodica seguida de suavizacao por serie de Fourier
x = kp(:,1)/100; v = kp(:,2);
xx = [x(1:end-1)-1; x(1:end-1); x+1];
vv = [v(1:end-1); v(1:end-1); v];
yd = pchip(xx, vv, ph);
% manter 8 harmonicos -> curva suave e periodica
Y = fft(yd); K = 8;
Y(K+2:end-K) = 0;
y = real(ifft(Y));
end
