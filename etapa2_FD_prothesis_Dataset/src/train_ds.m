function train_ds(varargin)
%TRAIN_DS ETAPA 2: SAC controlando as excitacoes dos 15 musculos para
%   reproduzir a marcha saudavel do dataset (Fukuchi et al. 2018).
%
%   Curriculo de autonomia: e = beta*(e_base + 0.25u) + (1-beta)*(u+1)/2.
%   beta comeca em 1 (residuo, como na etapa 1) e cai 'betaStep' sempre que
%   a avaliacao deterministica anda os 10 s; volta a subir um pouco se a
%   avaliacao cair antes de 5 s. Objetivo: beta = 0 (controle muscular
%   direto, sem controlador-base).
%
%   Inicializacao: por padrao as redes partem do melhor agente da etapa 1
%   (apenas leitura; nada e gravado na pasta da etapa 1). 'init','scratch'
%   cria redes novas.
%
%   Tudo e salvo em etapa2_FD_prothesis_Dataset/{agents,results,logs}.
%   train_ds('nChunks',120,'workers',4,'resume',true)

p = inputParser;
p.addParameter('nChunks', 120);
p.addParameter('episodesPerChunk', 40);
p.addParameter('workers', 4);
p.addParameter('resume', true);
p.addParameter('init', 'etapa1');      % 'etapa1' | 'scratch' | caminho de um .mat com 'agent'
p.addParameter('betaStep', 0.1);
p.addParameter('betaBack', 0.05);
p.addParameter('evalEvery', 2);
p.addParameter('learnEvery', 4);
p.addParameter('lr', 1e-4);
p.parse(varargin{:}); o = p.Results;

here = fileparts(mfilename('fullpath'));
base = fullfile(here, '..');
agDir = fullfile(base, 'agents'); resDir = fullfile(base, 'results'); logDir = fullfile(base, 'logs');
for d = {agDir, resDir, logDir}, if ~exist(d{1}, 'dir'), mkdir(d{1}); end, end
ckpt = fullfile(agDir, 'sac_checkpoint.mat');
logFile = fullfile(logDir, 'train_log.csv');

cfg = gaitConfigDS();
env = DatasetGaitEnv(cfg);
obsInfo = getObservationInfo(env); actInfo = getActionInfo(env);

startChunk = 1; history = []; beta = 1; bestScore = inf;
if o.resume && isfile(ckpt)
    S = load(ckpt); agent = S.agent; startChunk = S.chunk + 1; history = S.history;
    beta = S.beta; bestScore = S.bestScore;
    fprintf('Retomando do bloco %d (beta=%.2f)\n', startChunk, beta);
else
    if strcmp(o.init, 'scratch')
        init = rlAgentInitializationOptions('NumHiddenUnit', 256);
        agent = rlSACAgent(obsInfo, actInfo, init);
        ao = agent.AgentOptions;
        ao.DiscountFactor = 0.99; ao.MiniBatchSize = 256; ao.ExperienceBufferLength = 1e6;
        ao.TargetSmoothFactor = 5e-3; ao.NumGradientStepsPerUpdate = 1;
        for i = 1:numel(ao.CriticOptimizerOptions), ao.CriticOptimizerOptions(i).GradientThreshold = 1; end
        ao.ActorOptimizerOptions.GradientThreshold = 1;
        ao.EntropyWeightOptions.TargetEntropy = -15; ao.EntropyWeightOptions.EntropyWeight = 0.05;
        agent.AgentOptions = ao;
        fprintf('Redes novas (scratch)\n');
    else
        if strcmp(o.init, 'etapa1')
            src = fullfile(base, '..', 'etapa1_FD_prothesis_Literature', 'results', 'best_agent.mat');
        else
            src = o.init;
        end
        S = load(src, 'agent'); agent = S.agent;
        fprintf('Redes iniciais copiadas de %s\n', src);
    end
    agent.AgentOptions.SampleTime = cfg.dt;
    agent.AgentOptions.NumWarmStartSteps = 2000;
end
agent.AgentOptions.LearningFrequency = o.learnEvery;
agent.AgentOptions.ResetExperienceBufferBeforeTraining = false;
ao = agent.AgentOptions;
ao.ActorOptimizerOptions.LearnRate = o.lr;
for i = 1:numel(ao.CriticOptimizerOptions), ao.CriticOptimizerOptions(i).LearnRate = o.lr; end
agent.AgentOptions = ao;

if o.workers > 0
    pool = gcp('nocreate');
    if isempty(pool) || pool.NumWorkers ~= o.workers
        delete(pool); pool = parpool('Processes', o.workers);
    end
    parfevalOnAll(pool, @addpath, 0, here);
end

for chunk = startChunk:o.nChunks
    env.cfg.baseWeight = beta;
    topts = rlTrainingOptions('MaxEpisodes', o.episodesPerChunk, 'MaxStepsPerEpisode', cfg.maxSteps, ...
        'ScoreAveragingWindowLength', 20, 'StopTrainingCriteria', 'none', ...
        'Plots', 'none', 'Verbose', false, 'UseParallel', o.workers > 0);
    if o.workers > 0, topts.ParallelizationOptions.Mode = 'async'; end
    tic; stats = train(agent, env, topts); el = toc;
    R = stats.EpisodeReward; L = stats.EpisodeSteps;
    % colunas: bloco beta Rmed Rmax passosMed passosMax tempo | aval: sobrev v score inBand apoioD apoioE
    row = [chunk, beta, mean(R), max(R), mean(L), max(L), el, NaN(1, 6)];
    history = [history; row]; %#ok<AGROW>
    fprintf('[%s] bloco %d/%d | beta=%.2f | R med=%.1f max=%.1f | passos med=%.0f max=%d | %.0f s\n', ...
        datestr(now, 'HH:MM:SS'), chunk, o.nChunks, beta, mean(R), max(R), mean(L), max(L), el);

    if mod(chunk, o.evalEvery) == 0 || chunk == o.nChunks
        tag = sprintf('chunk%03d_b%03d', chunk, round(100*beta));
        M = evaluate_ds(agent, 'baseWeight', beta, 'duration', 10, 'tag', tag, 'quiet', true);
        fprintf('   AVAL beta=%.2f: %.2f s | v=%.2f m/s | score(z)=%.2f | dentro 1DP=%.0f%% | apoio D/E=%.0f/%.0f%% (saudavel %.0f%%)\n', ...
            beta, M.survivalTime, M.meanSpeed, M.score, M.inBand, M.stance.R, M.stance.L, M.refStance);
        history(end, 8:13) = [M.survivalTime, M.meanSpeed, M.score, M.inBand, M.stance.R, M.stance.L];
        if M.survivalTime >= 0.99*10
            save(fullfile(agDir, sprintf('agent_%s.mat', tag)), 'agent', 'M', 'beta');
            if beta == 0 && M.score < bestScore
                bestScore = M.score;
                save(fullfile(agDir, 'best_agent_direct.mat'), 'agent', 'M', 'beta', 'chunk');
                fprintf('   >> novo melhor agente com controle direto (score %.2f)\n', bestScore);
            end
            beta = max(0, round(100*(beta - o.betaStep))/100);
        elseif M.survivalTime < 5
            beta = min(1, round(100*(beta + o.betaBack))/100);
        end
    end
    save(ckpt, 'agent', 'chunk', 'history', 'beta', 'bestScore');
    writematrix(history, logFile);
end
end
