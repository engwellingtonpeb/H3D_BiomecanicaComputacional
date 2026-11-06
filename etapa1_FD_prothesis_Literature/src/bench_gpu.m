% Mede o custo do APRENDIZ do SAC (gradientes das redes) em CPU vs GPU,
% isolado da simulacao: usa um ambiente falso instantaneo com as mesmas
% dimensoes do ProstheticGaitEnv (obs 63, acao 15) e o mesmo agente.
obsInfo = rlNumericSpec([63 1]);
actInfo = rlNumericSpec([15 1], 'LowerLimit', -1, 'UpperLimit', 1);
env = rlFunctionEnv(obsInfo, actInfo, @(a, lg) deal(randn(63,1), rand, false, lg), @() deal(randn(63,1), []));
nSteps = 400; warm = 300;
for dev = ["cpu" "gpu"]
    agent = rlSACAgent(obsInfo, actInfo, rlAgentInitializationOptions('NumHiddenUnit', 256));
    ao = agent.AgentOptions;
    ao.MiniBatchSize = 256; ao.NumWarmStartSteps = warm; ao.LearningFrequency = 1;
    agent.AgentOptions = ao;
    actor = getActor(agent); actor.UseDevice = dev; agent = setActor(agent, actor);
    critics = getCritic(agent); for i = 1:numel(critics), critics(i).UseDevice = dev; end
    agent = setCritic(agent, critics);
    topts = rlTrainingOptions('MaxEpisodes', 1, 'MaxStepsPerEpisode', warm, 'Plots', 'none', 'Verbose', false, 'StopTrainingCriteria', 'none');
    train(agent, env, topts);                        % aquece (preenche buffer, compila)
    topts.MaxStepsPerEpisode = nSteps;
    tic; train(agent, env, topts); el = toc;
    fprintf('%s: %.2f ms por passo com 1 gradiente/passo (%d passos em %.1f s)\n', dev, 1000*el/nSteps, nSteps, el);
end
