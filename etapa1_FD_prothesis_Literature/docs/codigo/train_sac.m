function train_sac(varargin)
%TRAIN_SAC Treina um agente SAC que modula as excitacoes musculares e os
%   atuadores da protese. Treino em blocos: entre blocos a assistencia na
%   pelve e reduzida (curriculo), o agente e salvo e avaliado sem assistencia.
%
%   train_sac('nChunks',60,'episodesPerChunk',40,'workers',4,'resume',true)

p = inputParser;
p.addParameter('nChunks', 150);
p.addParameter('episodesPerChunk', 40);
p.addParameter('workers', 4);
p.addParameter('resume', true);
p.addParameter('assistInit', 0);         % 0 = sem assistencia (recomendado: o sinal de queda guia o aprendizado)
p.addParameter('assistFullChunks', 0);
p.addParameter('assistStepDown', 0.05);  % reducao da assistencia por bloco bem-sucedido
p.addParameter('assistStepUp', 0);       % >0 reativa assistencia se cair cedo
p.addParameter('evalEvery', 3);
p.addParameter('learnEvery', 4);
p.addParameter('lr', []);              % [] = manter a do agente (3e-4 no inicio)
p.addParameter('device', 'cpu');       % 'gpu' = redes do aprendiz na GPU. Medido (bench_gpu.m, RTX 2060):
                                       % 177 ms/gradiente na CPU vs 188 na GPU -> sem ganho com redes de 256
p.parse(varargin{:}); o = p.Results;

here = fileparts(mfilename('fullpath'));
outDir = fullfile(here, '..', 'results'); if ~exist(outDir, 'dir'), mkdir(outDir); end
ckpt = fullfile(outDir, 'sac_checkpoint.mat');
logFile = fullfile(outDir, 'train_log.csv');

cfg = gaitConfig();
env = ProstheticGaitEnv(cfg);
obsInfo = getObservationInfo(env); actInfo = getActionInfo(env);

startChunk = 1; history = [];
if o.resume && isfile(ckpt)
    S = load(ckpt); agent = S.agent; startChunk = S.chunk + 1; history = S.history;
    fprintf('Retomando do bloco %d\n', startChunk);
else
    init = rlAgentInitializationOptions('NumHiddenUnit', 256);
    agent = rlSACAgent(obsInfo, actInfo, init);
    ao = agent.AgentOptions;
    ao.SampleTime = cfg.dt;
    ao.DiscountFactor = 0.99;
    ao.MiniBatchSize = 256;
    ao.ExperienceBufferLength = 1e6;
    ao.TargetSmoothFactor = 5e-3;
    ao.NumWarmStartSteps = 2000;
    ao.LearningFrequency = 1;       % gradiente a cada passo
    ao.NumGradientStepsPerUpdate = 1;
    ao.ActorOptimizerOptions.LearnRate = 3e-4;
    for i = 1:numel(ao.CriticOptimizerOptions)
        ao.CriticOptimizerOptions(i).LearnRate = 3e-4;
        ao.CriticOptimizerOptions(i).GradientThreshold = 1;
    end
    ao.ActorOptimizerOptions.GradientThreshold = 1;
    ao.EntropyWeightOptions.TargetEntropy = -15;
    ao.EntropyWeightOptions.EntropyWeight = 0.05;
    agent.AgentOptions = ao;
end
% aprendiz no processo principal e o gargalo: 1 atualizacao a cada
% o.learnEvery passos (aplicado tambem ao retomar de checkpoint)
agent.AgentOptions.LearningFrequency = o.learnEvery;
agent.AgentOptions.ResetExperienceBufferBeforeTraining = false;   % manter buffer entre blocos
if ~isempty(o.lr)   % taxa de aprendizado (ator e criticos); menor = politica mais estavel
    ao = agent.AgentOptions;
    ao.ActorOptimizerOptions.LearnRate = o.lr;
    for i = 1:numel(ao.CriticOptimizerOptions), ao.CriticOptimizerOptions(i).LearnRate = o.lr; end
    agent.AgentOptions = ao;
    fprintf('Taxa de aprendizado = %g\n', o.lr);
end
actor = getActor(agent); actor.UseDevice = o.device; agent = setActor(agent, actor);
critics = getCritic(agent); for i = 1:numel(critics), critics(i).UseDevice = o.device; end
agent = setCritic(agent, critics);
fprintf('Aprendiz em: %s\n', o.device);

if o.workers > 0
    pool = gcp('nocreate');
    if isempty(pool) || pool.NumWorkers ~= o.workers
        delete(pool); pool = parpool('Processes', o.workers);
    end
    addAttachedFiles(pool, {fullfile(here, 'ProstheticGaitEnv.m'), fullfile(here, 'gaitConfig.m'), fullfile(here, 'healthyGaitReference.m')});
    parfevalOnAll(pool, @addpath, 0, here);
end

% curriculo ADAPTATIVO de assistencia: so reduz quando o agente aguenta
% >= 80% do episodio; aumenta um pouco se cair antes de 50%.
if isempty(history), a = o.assistInit; else, a = history(end, 2); end
for chunk = startChunk:o.nChunks
    if ~isempty(history) && chunk > o.assistFullChunks
        fracSurv = history(end, 5) / cfg.maxSteps;
        if fracSurv >= 0.8,     a = max(0, a - o.assistStepDown);
        elseif fracSurv < 0.5,  a = min(1, a + o.assistStepUp);
        end
        if a < 0.03, a = 0; end
    end
    env.cfg.assistStart = a; env.cfg.assistDecayEpisodes = 0;

    topts = rlTrainingOptions( ...
        'MaxEpisodes', o.episodesPerChunk, ...
        'MaxStepsPerEpisode', cfg.maxSteps, ...
        'ScoreAveragingWindowLength', 20, ...
        'StopTrainingCriteria', 'none', ...
        'Plots', 'none', 'Verbose', false, ...
        'UseParallel', o.workers > 0);
    if o.workers > 0, topts.ParallelizationOptions.Mode = 'async'; end

    tic;
    stats = train(agent, env, topts);
    el = toc;
    R = stats.EpisodeReward; L = stats.EpisodeSteps;
    row = [chunk, a, mean(R), max(R), mean(L), max(L), el, NaN, NaN, NaN];
    history = [history; row]; %#ok<AGROW>
    fprintf('[%s] bloco %d/%d | assist=%.2f | R med=%.1f max=%.1f | passos med=%.0f max=%d | %.0f s\n', ...
        datestr(now, 'HH:MM:SS'), chunk, o.nChunks, a, mean(R), max(R), mean(L), max(L), el);

    if mod(chunk, o.evalEvery) == 0 || chunk == o.nChunks
        M = evaluate_policy(agent, 'assist', 0, 'duration', 10, 'tag', sprintf('chunk%03d', chunk), 'quiet', true);
        fprintf('   AVAL (sem assistencia): %.2f s em pe | v=%.2f m/s | RMSE=%.1f graus | corr=%.2f | simetria=%.0f%% | jerk=%.2f\n', ...
            M.survivalTime, M.meanSpeed, M.rmseMean, M.corrMean, M.stanceSymmetry, M.jerkIndex);
        history(end, 8:10) = [M.survivalTime, M.rmseMean, M.corrMean];
        if M.survivalTime >= 0.99*10, save(fullfile(outDir, sprintf('agent_walk_chunk%03d.mat', chunk)), 'agent', 'M'); end
    end
    save(ckpt, 'agent', 'chunk', 'history', 'cfg');
    writematrix(history, logFile);
end
end
