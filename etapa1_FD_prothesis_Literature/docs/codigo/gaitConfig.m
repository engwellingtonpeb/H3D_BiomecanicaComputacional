function c = gaitConfig(varargin)
%GAITCONFIG Parametros do ambiente, controlador-base e recompensa.
%   c = gaitConfig('planar', false, 'assistDecayEpisodes', 3000, ...)

here = fileparts(mfilename('fullpath'));
c.modelFile = fullfile(here, '..', 'OpenSIM', 'OS4_gait14dof15musc_2Act_RTFP_DynAct.osim');

c.planar = true;     % estagio 1: marcha restrita ao plano sagital (+ add/abd do quadril)
c.dt = 0.01;         % passo de controle (s)
c.maxSteps = 600;    % 6 s por episodio (~5.5 ciclos; era 3 s ate o bloco 93)
c.odeSolver = @ode23;   % integrador do MATLAB (ode23 foi o mais rapido; ode15s/ode23t p/ casos rigidos)
c.odeRelTol = 1e-3; c.odeAbsTol = 1e-4;
c.momentArmEvery = 3;

c.muscles = {'abd_r','add_r','glut_max_r','iliopsoas_r', ...
             'abd_l','add_l','hamstrings_l','bifemsh_l','glut_max_l','iliopsoas_l', ...
             'rect_fem_l','vasti_l','gastroc_l','soleus_l','tib_ant_l'};

c.allCoords = {'pelvis_tilt','pelvis_list','pelvis_rotation','pelvis_tx','pelvis_ty','pelvis_tz', ...
               'hip_flexion_r','hip_adduction_r','hip_rotation_r','knee_angle_r','ankle_angle_r', ...
               'hip_flexion_l','hip_adduction_l','knee_angle_l','ankle_angle_l'};

% juntas controladas pelos musculos (controlador-base)
c.ctrlJoints = {'hip_flexion_r','hip_adduction_r','hip_flexion_l','hip_adduction_l','knee_angle_l','ankle_angle_l'};
c.trackJoints = {'hip_flexion_r','knee_angle_r','ankle_angle_r','hip_flexion_l','knee_angle_l','ankle_angle_l','hip_adduction_r','hip_adduction_l'};
c.trackW      = [1 1 1 1 1 1 0.3 0.3];

c.targetSpeed = 1.2;      % m/s
c.pelvisTiltRef = 0;      % rad
c.initPhase = 0;
c.randomInitPhase = true;
c.initPenetration = 0.004;

% ganhos do controlador-base (Nm/rad, Nm s/rad)
c.Kp = [250 150 250 150 200 150];
c.Kd = [ 15  10  15  10  12   8];
c.KpTorso = 1500; c.KdTorso = 100;
c.lambda = 1e-4; c.baseExcitation = 0.02;

% PROTESE PASSIVA (sem controle): mola torcional + amortecedor viscoso
% joelho: tipo mecanico com fricc. hidraulica simples + leve mola de extensao
% tornozelo: pe de armazenamento de energia (ESAR), ~6 Nm/grau p/ ~75 kg
c.prosthesis.knee  = struct("coord","knee_angle_r",  "stiffness", 5,   "rest", 0, "damping", 3);
c.prosthesis.ankle = struct("coord","ankle_angle_r", "stiffness", 350, "rest", 0, "damping", 2);

% acao do agente
c.residualScale = 0.25;

c.actionFilter = 0.5;     % u = f*u_prev + (1-f)*a

% assistencia na pelve (curriculo) -- removida gradualmente
c.assistCoords = {'pelvis_tilt','pelvis_tx','pelvis_ty','pelvis_list'};
c.assistKp = [400 0 3000 300];
c.assistKd = [40 300 300 30];
c.assistNorm = [100 100 300 100];
c.assistStart = 0;
c.assistDecayEpisodes = 0;   % 0 = assistencia fixa em c.assistStart

% terminacao
c.minPelvisHeight = 0.70;
c.maxTilt = 0.6;
c.fallPenalty = 2;

% recompensa (imitacao + tarefa + regularizacao fisiologica)
c.wPose = 0.45; c.kPose = 3;
c.wVel = 0.10;  c.kVel = 0.05;
c.wSpeed = 0.45; c.kSpeed = 2.5;   % mais peso e kernel mais estreito: 0.6 m/s ja e claramente pior que 1.2 (era 0.25/1.5 ate o bloco 93)
c.wUpright = 0.15; c.kUpright = 20; c.kHeight = 50;
c.wAlive = 0.30;
c.wEffort = 0.2; c.wSmooth = 2.0; c.wAssist = 0.2; c.wAction = 0.02;

for i = 1:2:numel(varargin), c.(varargin{i}) = varargin{i+1}; end

if c.planar
    c.lockedCoords = {'pelvis_list','pelvis_rotation','pelvis_tz','hip_rotation_r'};
    keep = ~ismember(c.assistCoords, c.lockedCoords);
    c.assistCoords = c.assistCoords(keep); c.assistKp = c.assistKp(keep);
    c.assistKd = c.assistKd(keep); c.assistNorm = c.assistNorm(keep);
else
    c.lockedCoords = {'hip_rotation_r'};
end
c.obsCoords = setdiff(c.allCoords, [c.lockedCoords, {'hip_rotation_l','lumbar_extension'}], 'stable');

% Fmax e mascara musculo x junta
c.Fmax = muscleFmax(c);
c.muscleJointMask = muscleJointMask(c);
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

function M = muscleJointMask(c)
% quais musculos cruzam quais juntas (anatomia do modelo)
map = struct( ...
  'hip_flexion_r',   {{'abd_r','add_r','glut_max_r','iliopsoas_r'}}, ...
  'hip_adduction_r', {{'abd_r','add_r','glut_max_r','iliopsoas_r'}}, ...
  'hip_flexion_l',   {{'abd_l','add_l','hamstrings_l','glut_max_l','iliopsoas_l','rect_fem_l'}}, ...
  'hip_adduction_l', {{'abd_l','add_l','hamstrings_l','glut_max_l','iliopsoas_l','rect_fem_l'}}, ...
  'knee_angle_l',    {{'hamstrings_l','bifemsh_l','rect_fem_l','vasti_l','gastroc_l'}}, ...
  'ankle_angle_l',   {{'gastroc_l','soleus_l','tib_ant_l'}});
M = false(numel(c.ctrlJoints), numel(c.muscles));
for j = 1:numel(c.ctrlJoints)
    M(j,:) = ismember(c.muscles, map.(c.ctrlJoints{j}));
end
end
