function c = gaitConfigDS(varargin)
%GAITCONFIGDS Parametros da ETAPA 2: o agente controla diretamente as
%   excitacoes dos 15 musculos do modelo e e recompensado por reproduzir a
%   marcha saudavel MEDIDA (dataset Fukuchi et al. 2018), com tolerancia
%   dada pela variabilidade entre sujeitos (+/- 1 DP).
%
%   Mesmo modelo OpenSim e mesma dinamica direta da etapa 1 (OpenSim avalia
%   ydot, ode23 do MATLAB integra), mesma protese passiva.
%
%   c = gaitConfigDS('baseWeight', 0.5, ...)

here = fileparts(mfilename('fullpath'));
c.modelFile = fullfile(here, '..', '..', 'OpenSIM', 'OS4_gait14dof15musc_2Act_RTFP_DynAct.osim');

c.planar = true;
c.dt = 0.01;
c.maxSteps = 600;        % 6 s = 5 passadas de 1,2 s
c.odeSolver = @ode23;
c.odeRelTol = 1e-3; c.odeAbsTol = 1e-4;
c.momentArmEvery = 3;

% TODOS os musculos do modelo (15; a protese nao tem musculos)
c.muscles = {'abd_r','add_r','glut_max_r','iliopsoas_r', ...
             'abd_l','add_l','hamstrings_l','bifemsh_l','glut_max_l','iliopsoas_l', ...
             'rect_fem_l','vasti_l','gastroc_l','soleus_l','tib_ant_l'};

c.allCoords = {'pelvis_tilt','pelvis_list','pelvis_rotation','pelvis_tx','pelvis_ty','pelvis_tz', ...
               'hip_flexion_r','hip_adduction_r','hip_rotation_r','knee_angle_r','ankle_angle_r', ...
               'hip_flexion_l','hip_adduction_l','knee_angle_l','ankle_angle_l'};
c.ctrlJoints = {'hip_flexion_r','hip_adduction_r','hip_flexion_l','hip_adduction_l','knee_angle_l','ankle_angle_l'};
c.trackJoints = {'hip_flexion_r','knee_angle_r','ankle_angle_r','hip_flexion_l','knee_angle_l','ankle_angle_l','hip_adduction_r','hip_adduction_l'};
c.trackW      = [1 0.5 0.5 1 1 1 0.3 0.3];   % joelho/tornozelo proteticos sao passivos: peso menor

% referencia: classe de velocidade do dataset ('S' lenta 0,86 m/s; 'C' confortavel 1,27 m/s)
c.refClass = 'S';
c.pelvisTiltRef = 0;
c.initPhase = 0;
c.randomInitPhase = true;
c.initPenetration = 0.004;

% controlador-base (so como andaime no inicio do curriculo)
c.Kp = [250 150 250 150 200 150];
c.Kd = [ 15  10  15  10  12   8];
c.KpTorso = 1500; c.KdTorso = 100;
c.lambda = 1e-4; c.baseExcitation = 0.02;

% protese passiva (identica a etapa 1)
c.prosthesis.knee  = struct("coord","knee_angle_r",  "stiffness", 5,   "rest", 0, "damping", 3);
c.prosthesis.ankle = struct("coord","ankle_angle_r", "stiffness", 350, "rest", 0, "damping", 2);

% ACAO: e = beta*(e_base + 0.25*u) + (1-beta)*(u+1)/2
%   beta = 1 -> residuo sobre o controlador-base (como na etapa 1)
%   beta = 0 -> o agente define sozinho a excitacao de cada musculo
%   beta e reduzido pelo treino so quando a avaliacao anda os 10 s.
c.baseWeight = 1;
c.residualScale = 0.25;
c.actionFilter = 0.5;

% sem assistencia na pelve
c.assistCoords = {'pelvis_tilt','pelvis_tx','pelvis_ty','pelvis_list'};
c.assistKp = [400 0 3000 300]; c.assistKd = [40 300 300 30]; c.assistNorm = [100 100 300 100];
c.assistStart = 0; c.assistDecayEpisodes = 0;

% terminacao
c.minPelvisHeight = 0.70;
c.maxTilt = 0.6;
c.fallPenalty = 2;

% RECOMPENSA: erros normalizados pelo DP entre sujeitos (z = erro/DP)
c.wPose = 0.40;  c.kPose = 0.3;   c.sdFloorDeg = 2;      % pose = exp(-kPose * media_w(z^2))
c.wVel = 0.05;   c.kVel = 0.05;
c.wGrf = 0.25;   c.kGrf = 0.1;    c.sdFloorGrfv = 0.10; c.sdFloorGrfap = 0.05;   % BW
c.wSpeed = 0.30; c.kSpeed = 2.5;
c.wUpright = 0.10; c.kUpright = 20; c.kHeight = 50;
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
map = struct( ...
  'hip_flexion_r',   {{'abd_r','add_r','glut_max_r','iliopsoas_r'}}, ...
  'hip_adduction_r', {{'abd_r','add_r','glut_max_r','iliopsoas_r'}}, ...
  'hip_flexion_l',   {{'abd_l','add_l','hamstrings_l','glut_max_l','iliopsoas_l','rect_fem_l'}}, ...
  'hip_adduction_l', {{'abd_l','add_l','hamstrings_l','glut_max_l','iliopsoas_l','rect_fem_l'}}, ...
  'knee_angle_l',    {{'hamstrings_l','bifemsh_l','rect_fem_l','vasti_l','gastroc_l'}}, ...
  'ankle_angle_l',   {{'gastroc_l','soleus_l','tib_ant_l'}});
M = false(numel(c.ctrlJoints), numel(c.muscles));
for j = 1:numel(c.ctrlJoints), M(j,:) = ismember(c.muscles, map.(c.ctrlJoints{j})); end
end
