addpath('C:/Users/engwe/Desktop/RL_OSim/etapa2_FD_prothesis_Dataset/src'); cd('C:/Users/engwe/Desktop/RL_OSim/etapa2_FD_prothesis_Dataset/src');
S = load('../../etapa1_FD_prothesis_Literature/results/best_agent.mat'); ag = S.agent;
for b = [1 0.9 0.5 0]
  M = evaluate_ds(ag, 'baseWeight', b, 'duration', 6, 'tag', sprintf('smoke_b%03d', round(100*b)));
end
M = evaluate_ds([], 'baseWeight', 1, 'duration', 6, 'tag', 'smoke_baseline');
env = DatasetGaitEnv(gaitConfigDS('randomInitPhase', false)); o = reset(env);
for k = 1:30, [o, r, d, info] = step(env, zeros(15,1)); end
disp(info); fprintf('r=%.3f\n', r);
