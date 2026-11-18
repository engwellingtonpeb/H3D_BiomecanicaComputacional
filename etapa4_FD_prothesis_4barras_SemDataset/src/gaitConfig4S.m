function c = gaitConfig4S(varargin)
%GAITCONFIG4S Parametros da ETAPA 4: protese com joelho POLICENTRICO de 4
%   barras (OpenSIM/50_Direita_Final.osim), SAC comandando diretamente as
%   excitacoes dos 15 musculos, SEM DATASET.
%
%   Nenhum dado de marcha saudavel entra no treino:
%     - sem controlador-base (ele seguia a trajetoria do dataset): beta = 0
%     - observacao sem a trajetoria de referencia
%     - recompensa so com criterios de plausibilidade fisiologica:
%       velocidade de caminhada, custo metabolico (excitacao^2 ponderada pela
%       forca maxima), tronco ereto, altura da pelve, amplitudes articulares
%       fisiologicas, pico de GRF, sem fase de voo e passos alternados
%       (com bonus de simetria entre os passos dos dois lados)
%   Estados iniciais: sorteados de marchas SIMULADAS na etapa 3 (nao do
%   dataset). Redes iniciais: agente da etapa 3 com controle direto.
%
%   c = gaitConfig4S('vLo', 1.0, ...)

here = fileparts(mfilename('fullpath'));
c.modelFile = fullfile(here, '..', '..', 'OpenSIM', '50_Direita_Final.osim');
c.fourBar.linkCoord = 'knee_angle_r';
c.fourBar.depCoords = {'dist_4_1_rz','4x2_dist_rz'};
c.fourBar.mapFile = fullfile(here, '..', 'data', 'fourbar_map.mat');
c.initStatesFile = fullfile(here, '..', 'data', 'init_states.mat');

c.planar = true;
c.dt = 0.01;
c.maxSteps = 800;        % 8 s
c.odeSolver = @ode23;
c.odeRelTol = 1e-3; c.odeAbsTol = 1e-4;

c.muscles = {'abd_r','add_r','glut_max_r','iliopsoas_r', ...
             'abd_l','add_l','hamstrings_l','bifemsh_l','glut_max_l','iliopsoas_l', ...
             'rect_fem_l','vasti_l','gastroc_l','soleus_l','tib_ant_l'};
c.allCoords = {'pelvis_tilt','pelvis_list','pelvis_rotation','pelvis_tx','pelvis_ty','pelvis_tz', ...
               'hip_flexion_r','hip_adduction_r','hip_rotation_r','knee_angle_r','ankle_angle_r', ...
               'hip_flexion_l','hip_adduction_l','knee_angle_l','ankle_angle_l'};
% 8 entradas que na etapa 3 levavam a referencia do dataset: aqui ficam em
% zero (mantem o tamanho da observacao para reaproveitar as redes)
c.nRefSlots = 8;
% relogio interno (oscilador tipo CPG, periodo livre) nas 2 entradas de fase;
% 'clock' = false -> entradas em zero (politica puramente por realimentacao)
c.clock = false; c.clockPeriod = 1.2;
c.initPenetration = 0.004;
c.initJitter = 0.02;     % ruido (rad, rad/s*10) no estado inicial sorteado

% protese passiva (mesma da etapa 3): mola + amortecedor no elo 4x1 e no tornozelo
c.prosthesis.knee  = struct("coord","knee_angle_r",  "stiffness", 5,   "rest", 0, "damping", 3);
c.prosthesis.ankle = struct("coord","ankle_angle_r", "stiffness", 350, "rest", 0, "damping", 2);

% acao: e = (u+1)/2 com filtro passa-baixa (controle muscular direto)
c.actionFilter = 0.5;
c.minExcitation = 0.01;

% terminacao
c.minPelvisHeight = 0.70;
c.maxTilt = 0.6;
c.fallPenalty = 2;

% ---------------- RECOMPENSA (sem dataset) ----------------
% r = (wAlive + wSpeed*vel + wUpright*ereto + bonus_passo) * exp(-penalidades)
c.vLo = 0.9; c.vHi = 1.3; c.kSpeed = 8;   % faixa de velocidade de caminhada (m/s)
c.wAlive = 0.5; c.wSpeed = 0.6; c.wUpright = 0.3;
c.kUpright = 20; c.kTiltRate = 0.5; c.kHeight = 100; % tronco/pelve e altura da pelve (so abaixo de heightRef)
c.heightRef = 0.90;      % pelve abaixo disso (m) e penalizada: evita marcha agachada
% bonus por passo alternado: wStep * comprimento/compRef * fator de simetria
c.wStep = 8; c.stepLenRef = 0.6; c.symWidth = 0.25; c.minStepLen = 0.05;
c.grfThr = 20;           % N, deteccao de contato
% penalidades (no expoente)
c.wEffort = 3;           % excitacao^2 ponderada por Fmax (~ custo metabolico)
c.wSmooth = 2.0; c.wAction = 0.02;
c.wROM = 20;             % excesso alem da amplitude fisiologica (rad^2)
c.wImpact = 2; c.impactBW = 1.5;  % pico de GRF vertical acima de 1,5 PC
c.wFlight = 1;           % os dois pes fora do solo (caminhada nao tem voo)
% amplitudes articulares fisiologicas (graus) [min max]
c.rom.hip_flexion_r   = [-30 100];  c.rom.hip_flexion_l   = [-30 100];
c.rom.hip_adduction_r = [-20 20];   c.rom.hip_adduction_l = [-20 20];
c.rom.knee_angle_l    = [-120 0];   c.rom.ankle_angle_l   = [-30 25];
c.rom.knee_angle_r    = [-100 0];   % flexao real do joelho protetico

for i = 1:2:numel(varargin), c.(varargin{i}) = varargin{i+1}; end

if c.planar
    c.lockedCoords = {'pelvis_list','pelvis_rotation','pelvis_tz','hip_rotation_r'};
else
    c.lockedCoords = {'hip_rotation_r'};
end
c.obsCoords = setdiff(c.allCoords, [c.lockedCoords, {'hip_rotation_l','lumbar_extension'}], 'stable');
c.Fmax = muscleFmax(c);
end

function F = muscleFmax(c)
import org.opensim.modeling.*
persistent cache
if isempty(cache)
    m = Model(c.modelFile); ms = m.getMuscles();
    cache = containers.Map();
    for i = 0:ms.getSize()-1, cache(char(ms.get(i).getName())) = ms.get(i).getMaxIsometricForce(); end
end
F = cellfun(@(n) cache(n), c.muscles);
end
