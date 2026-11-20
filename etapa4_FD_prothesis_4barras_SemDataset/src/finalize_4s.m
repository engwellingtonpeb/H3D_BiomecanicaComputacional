function best = finalize_4s(varargin)
%FINALIZE_4S Ao fim do treino da etapa 4 (4 barras, sem dataset): avalia os
%   agentes salvos por 20 s a partir de varios estados iniciais do banco,
%   escolhe o melhor e grava o video no visualizador do OpenSim.
%
%   Ordenacao: 1) estados iniciais em que andou os 20 s;
%              2) recompensa media por passo de tempo (o proprio objetivo, maior melhor).
%   Saidas: results/agent_selection.csv, agents/best_agent_final.mat,
%           results/final_iXXX_traj.mat e results/final_iXXX_opensim.mp4
p = inputParser;
p.addParameter('inits', [1 250 500 750 1000]);
p.addParameter('fromChunk', 100);     % so agentes a partir deste bloco
p.addParameter('duration', 20);
p.addParameter('workers', 4);
p.addParameter('render', true);
p.parse(varargin{:}); o = p.Results;

here = fileparts(mfilename('fullpath'));
agDir = fullfile(here, '..', 'agents'); resDir = fullfile(here, '..', 'results');
f = dir(fullfile(agDir, 'agent_chunk*.mat'));
k = cellfun(@(n) sscanf(n, 'agent_chunk%d.mat'), {f.name});
f = f(k >= o.fromChunk);
names = [{f.name} {'best_agent.mat'}];
C = load(fullfile(agDir, 'sac_checkpoint.mat'));
agent = C.agent; finalName = sprintf('agent_final_chunk%03d.mat', C.chunk);
save(fullfile(agDir, finalName), 'agent');
if ~ismember(sprintf('agent_chunk%03d.mat', C.chunk), names), names{end+1} = finalName; end
files = fullfile(agDir, names);

jobs = [repelem(1:numel(files), numel(o.inits))' repmat((1:numel(o.inits))', numel(files), 1)];
pool = gcp('nocreate'); if isempty(pool), parpool('Processes', o.workers); end
parfevalOnAll(@addpath, 0, here);
R = cell(size(jobs, 1), 1);
parfor j = 1:size(jobs, 1)
    S = load(files{jobs(j,1)}, 'agent'); ini = o.inits(jobs(j,2));
    tag = sprintf('cand_%s_i%04d', erase(names{jobs(j,1)}, '.mat'), ini);
    M = evaluate_4s(S.agent, 'duration', o.duration, 'init', ini, 'tag', tag, 'quiet', true);
    R{j} = struct('agent', string(names{jobs(j,1)}), 'init', ini, 'tag', string(tag), 'surv', M.survivalTime, ...
        'v', M.meanSpeed, 'r', M.meanReward, 'nSteps', M.nSteps, 'stanceR', M.stance.R, 'stanceL', M.stance.L, ...
        'SIstepTime', M.SI.stepTime, 'SIstepLen', M.SI.stepLen, 'effort', M.effort);
end
T = struct2table([R{:}]');
T.ok = T.surv >= 0.99 * o.duration;
G = groupsummary(T, 'agent', {'sum', 'mean'}, {'ok', 'r', 'v', 'surv', 'nSteps', 'effort'});
G = sortrows(G, {'sum_ok', 'mean_r'}, {'descend', 'descend'});
writetable(T, fullfile(resDir, 'agent_selection_raw.csv'));
writetable(G, fullfile(resDir, 'agent_selection.csv'));
disp(G(1:min(10,height(G)), {'agent', 'sum_ok', 'mean_r', 'mean_v', 'mean_surv', 'mean_nSteps', 'mean_effort'}));

best = G.agent(1);
S = load(fullfile(agDir, best), 'agent'); agent = S.agent; %#ok<NASGU>
save(fullfile(agDir, 'best_agent_final.mat'), 'agent', 'best');
% trajetoria do melhor: estado inicial em que andou mais (empate -> maior r)
Tb = sortrows(T(T.agent == best, :), {'surv', 'r'}, {'descend', 'descend'});
finalTag = sprintf('final_i%04d', Tb.init(1));
for ext = {'_traj.mat', '_motion.mot'}
    copyfile(fullfile(resDir, [char(Tb.tag(1)) ext{1}]), fullfile(resDir, [finalTag ext{1}]));
end
fprintf('melhor: %s | init %d | %.1f s | v=%.2f m/s | r/passo=%.3f | passos=%d | SI tempo/comp.=%.0f/%.0f%%\n', best, Tb.init(1), ...
    Tb.surv(1), Tb.v(1), Tb.r(1), Tb.nSteps(1), Tb.SIstepTime(1), Tb.SIstepLen(1));
if o.render, render_opensim_4s(finalTag); end
end
