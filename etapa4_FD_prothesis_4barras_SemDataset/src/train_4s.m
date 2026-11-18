function train_4s(varargin)
%TRAIN_4S ETAPA 4: melhor marcha possivel com o joelho policentrico (4
%   barras) SEM DATASET, mantendo plausibilidade fisiologica.
%
%   SAC comanda diretamente as excitacoes dos 15 musculos (sem
%   controlador-base). Recompensa de gaitConfig4S/FreeGaitEnv: velocidade de
%   caminhada, tronco ereto, passos alternados e simetricos, menos custo
%   metabolico, amplitudes articulares fisiologicas, sem impacto excessivo
%   e sem fase de voo. Estados iniciais sorteados de marchas simuladas na
%   etapa 3. Redes iniciais: agente da etapa 3 com controle direto
%   (data/init_agent_etapa3_chunk116.mat).
%
%   Tudo e salvo em etapa4_FD_prothesis_4barras_SemDataset/{agents,results,logs}.
%   train_4s('nChunks',150,'workers',4,'resume',true)

p = inputParser;
p.addParameter('nChunks', 150);
p.addParameter('episodesPerChunk', 40);
p.addParameter('workers', 4);
p.addParameter('resume', true);
p.addParameter('init', 'etapa3');      % 'etapa3' | 'scratch' | caminho de um .mat com 'agent'
p.addParameter('evalEvery', 2);
p.addParameter('evalInits', [1 500]);  % estados do banco usados na avaliacao
p.addParameter('learnEvery', 4);
p.addParameter('lr', 1e-4);
p.parse(varargin{:}); o = p.Results;

here = fileparts(mfilename('fullpath'));
base = fullfile(here, '..');
agDir = fullfile(base, 'agents'); resDir = fullfile(base, 'results'); logDir = fullfile(base, 'logs');
for d = {agDir, resDir, logDir}, if ~exist(d{1}, 'dir'), mkdir(d{1}); end, end
ckpt = fullfile(agDir, 'sac_checkpoint.mat');
logFile = fullfile(logDir, 'train_log.csv');

cfg = gaitConfig4S();
if ~isfile(cfg.initStatesFile), build_init_states(); end
env = FreeGaitEnv(cfg);
obsInfo = getObservationInfo(env); actInfo = getActionInfo(env);

startChunk = 1; history = []; bestScore = -inf;
if o.resume && isfile(ckpt)
    S = load(ckpt); agent = S.agent; startChunk = S.chunk + 1; history = S.history; bestScore = S.bestScore;
    fprintf('Retomando do bloco %d\n', startChunk);
else
    if strcmp(o.init, 'scratch')
        init = rlAgentInitializationOptions('NumHiddenUnit', 256);
        agent = rlSACAgent(obsInfo, actInfo, init);
        ao = agent.AgentOptions;
        ao.DiscountFactor = 0.99; ao.MiniBatchSize = 256; ao.ExperienceBufferLength = 1e6;
        ao.TargetSmoothFactor = 5e-3;
        for i = 1:numel(ao.CriticOptimizerOptions), ao.CriticOptimizerOptions(i).GradientThreshold = 1; end
        ao.ActorOptimizerOptions.GradientThreshold = 1;
        ao.EntropyWeightOptions.TargetEntropy = -15; ao.EntropyWeightOptions.EntropyWeight = 0.05;
        agent.AgentOptions = ao;
        fprintf('Redes novas (scratch)\n');
    else
        if strcmp(o.init, 'etapa3'), src = fullfile(base, 'data', 'init_agent_etapa3_chunk116.mat'); else, src = o.init; end
        S = load(src, 'agent'); agent = S.agent;
        fprintf('Redes iniciais copiadas de %s\n', src);
    end
    agent.AgentOptions.SampleTime = cfg.dt;
    agent.AgentOptions.NumWarmStartSteps = 2000;
    % recompensa nova: o buffer de experiencias da etapa 3 nao serve
    agent.AgentOptions.ResetExperienceBufferBeforeTraining = true;
end
agent.AgentOptions.LearningFrequency = o.learnEvery;
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
    topts = rlTrainingOptions('MaxEpisodes', o.episodesPerChunk, 'MaxStepsPerEpisode', cfg.maxSteps, ...
        'ScoreAveragingWindowLength', 20, 'StopTrainingCriteria', 'none', ...
        'Plots', 'none', 'Verbose', false, 'UseParallel', o.workers > 0);
    if o.workers > 0, topts.ParallelizationOptions.Mode = 'async'; end
    tic; stats = train(agent, env, topts); el = toc;
    agent.AgentOptions.ResetExperienceBufferBeforeTraining = false;
    R = stats.EpisodeReward; L = stats.EpisodeSteps;
    % colunas: bloco Rmed Rmax passosMed passosMax tempo | aval: sobrev v r/passo nPassos apoioD apoioE SItempo SIcomp esforco
    row = [chunk, mean(R), max(R), mean(L), max(L), el, NaN(1, 9)];
    history = [history; row]; %#ok<AGROW>
    fprintf('[%s] bloco %d/%d | R med=%.1f max=%.1f | passos med=%.0f max=%d | %.0f s\n', ...
        datestr(now, 'HH:MM:SS'), chunk, o.nChunks, mean(R), max(R), mean(L), max(L), el);

    if mod(chunk, o.evalEvery) == 0 || chunk == o.nChunks
        tag = sprintf('chunk%03d', chunk); Ms = cell(numel(o.evalInits), 1);
        for j = 1:numel(o.evalInits)
            Ms{j} = evaluate_4s(agent, 'duration', 10, 'tag', sprintf('%s_i%d', tag, o.evalInits(j)), 'init', o.evalInits(j), 'quiet', true);
        end
        Ms = [Ms{:}];
        surv = mean([Ms.survivalTime]); score = mean([Ms.meanReward]) * min([Ms.survivalTime]) / 10;
        M = Ms(1);
        fprintf('   AVAL: %.2f s | v=%.2f m/s | r/passo=%.3f | passos alt.=%d | apoio D/E=%.0f/%.0f%% | SI tempo/comp. passo=%.0f/%.0f%% | esforco=%.3f\n', ...
            surv, mean([Ms.meanSpeed]), mean([Ms.meanReward]), M.nSteps, M.stance.R, M.stance.L, M.SI.stepTime, M.SI.stepLen, mean([Ms.effort]));
        history(end, 7:15) = [surv, mean([Ms.meanSpeed]), mean([Ms.meanReward]), M.nSteps, M.stance.R, M.stance.L, M.SI.stepTime, M.SI.stepLen, mean([Ms.effort])];
        if min([Ms.survivalTime]) >= 0.99 * 10
            save(fullfile(agDir, sprintf('agent_%s.mat', tag)), 'agent', 'Ms');
            if score > bestScore
                bestScore = score;
                save(fullfile(agDir, 'best_agent.mat'), 'agent', 'Ms', 'chunk');
                fprintf('   >> novo melhor agente (r/passo %.3f)\n', score);
            end
        end
    end
    save(ckpt, 'agent', 'chunk', 'history', 'bestScore');
    writematrix(history, logFile);
end
end
