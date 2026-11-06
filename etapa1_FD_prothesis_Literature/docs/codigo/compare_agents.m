function T = compare_agents(varargin)
%COMPARE_AGENTS Avaliacao robusta dos agentes salvos: cada agente anda
%   'duration' segundos a partir de varias fases iniciais do ciclo. Escolhe o
%   mais robusto (nao o que teve sorte numa unica avaliacao).
%
%   T = compare_agents()                         % todos results/agent_walk_*.mat
%   T = compare_agents('duration', 20, 'phases', [0 .25 .5 .75], 'workers', 4)

p = inputParser;
p.addParameter('duration', 20);
p.addParameter('phases', [0 0.25 0.5 0.75]);
p.addParameter('workers', 4);
p.addParameter('files', {});
p.parse(varargin{:}); o = p.Results;

here = fileparts(mfilename('fullpath'));
outDir = fullfile(here, '..', 'results');
files = o.files;
if isempty(files)
    d = dir(fullfile(outDir, 'agent_walk_*.mat')); files = {d.name};
end
nA = numel(files); nP = numel(o.phases);
fprintf('%d agentes x %d fases x %g s\n', nA, nP, o.duration);

if o.workers > 0
    pool = gcp('nocreate');
    if isempty(pool), pool = parpool('Processes', o.workers); end
    addAttachedFiles(pool, {fullfile(here, 'ProstheticGaitEnv.m'), fullfile(here, 'gaitConfig.m'), ...
        fullfile(here, 'healthyGaitReference.m'), fullfile(here, 'evaluate_policy.m')});
    parfevalOnAll(pool, @addpath, 0, here);
end

jobs = [repelem((1:nA)', nP), repmat((1:nP)', nA, 1)];
R = nan(size(jobs, 1), 7);   % surv, v, rmse, corr, sym, jerk, plausible
parfor (j = 1:size(jobs, 1), o.workers)   % workers = 0 -> serial, sem abrir pool
    a = jobs(j, 1); ph = o.phases(jobs(j, 2));
    S = load(fullfile(outDir, files{a}), 'agent');
    [~, nm] = fileparts(files{a});
    tag = sprintf('cmp_%s_p%02d', erase(nm, 'agent_walk_'), round(100 * ph));
    M = evaluate_policy(S.agent, 'assist', 0, 'duration', o.duration, 'initPhase', ph, 'tag', tag, 'quiet', true);
    R(j, :) = [M.survivalTime, M.meanSpeed, M.rmseMean, M.corrMean, M.stanceSymmetry, M.jerkIndex, M.plausible];
end

T = table();
for a = 1:nA
    r = R(jobs(:, 1) == a, :);
    full = r(:, 1) >= 0.99 * o.duration;
    T(a, :) = table(string(files{a}), mean(r(:,1)), sum(full), mean(r(:,2)), mean(r(:,3)), mean(r(:,4)), ...
        mean(r(:,5)), mean(r(:,6)), sum(r(:,7)), ...
        'VariableNames', {'agente','tempoMedio_s','fasesCompletas','v_ms','RMSE_graus','corr','simetria_pct','jerk','plausivel'});
end
% robustez primeiro, depois qualidade da marcha
T = sortrows(T, {'fasesCompletas','tempoMedio_s','corr'}, {'descend','descend','descend'});
disp(T);
writetable(T, fullfile(outDir, 'agent_comparison.csv'));
writematrix([jobs R], fullfile(outDir, 'agent_comparison_raw.csv'));
end
