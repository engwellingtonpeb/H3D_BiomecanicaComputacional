function best = finalize_ds(varargin)
%FINALIZE_DS Ao fim do treino da etapa 2: avalia os agentes com controle
%   muscular direto (beta = 0) por 20 s a partir de varias fases iniciais,
%   escolhe o melhor e grava o video no visualizador do OpenSim.
%
%   Ordenacao: 1) fases em que andou os 20 s; 2) score z medio (menor melhor).
%   Saidas: results/agent_selection.csv, agents/best_agent_final.mat,
%           results/final_pXX_traj.mat e results/final_pXX_opensim.mp4
p = inputParser;
p.addParameter('phases', [0 0.5]);
p.addParameter('duration', 20);
p.addParameter('workers', 4);
p.addParameter('render', true);
p.parse(varargin{:}); o = p.Results;

here = fileparts(mfilename('fullpath'));
agDir = fullfile(here, '..', 'agents'); resDir = fullfile(here, '..', 'results');
f = [dir(fullfile(agDir, 'agent_chunk*_b000.mat')); dir(fullfile(agDir, 'best_agent_direct.mat'))];
names = {f.name}; files = fullfile({f.folder}, names);
% agente final do treino, se terminou com beta = 0
C = load(fullfile(agDir, 'sac_checkpoint.mat'));
if C.beta == 0
    agent = C.agent; save(fullfile(agDir, sprintf('agent_final_chunk%03d.mat', C.chunk)), 'agent');
    names{end+1} = sprintf('agent_final_chunk%03d.mat', C.chunk); files{end+1} = fullfile(agDir, names{end});
end
jobs = [repelem(1:numel(files), numel(o.phases))' repmat((1:numel(o.phases))', numel(files), 1)];
pool = gcp('nocreate'); if isempty(pool), parpool('Processes', o.workers); end
parfevalOnAll(@addpath, 0, here);
R = cell(size(jobs, 1), 1);
parfor j = 1:size(jobs, 1)
    S = load(files{jobs(j,1)}, 'agent'); ph = o.phases(jobs(j,2));
    tag = sprintf('cand_%s_p%02d', erase(names{jobs(j,1)}, '.mat'), round(100*ph));
    M = evaluate_ds(S.agent, 'baseWeight', 0, 'duration', o.duration, 'initPhase', ph, 'tag', tag, 'quiet', true);
    R{j} = struct('agent', string(names{jobs(j,1)}), 'phase', ph, 'tag', string(tag), 'surv', M.survivalTime, ...
        'v', M.meanSpeed, 'score', M.score, 'inBand', M.inBand, 'stanceR', M.stance.R, 'stanceL', M.stance.L);
end
T = struct2table([R{:}]');
T.ok = T.surv >= 0.99 * o.duration;
G = groupsummary(T, 'agent', {'sum', 'mean'}, {'ok', 'score', 'inBand', 'v', 'surv'});
G = sortrows(G, {'sum_ok', 'mean_score'}, {'descend', 'ascend'});
writetable(T, fullfile(resDir, 'agent_selection_raw.csv'));
writetable(G, fullfile(resDir, 'agent_selection.csv'));
disp(G(:, {'agent', 'sum_ok', 'mean_score', 'mean_inBand', 'mean_v', 'mean_surv'}));

best = G.agent(1);
S = load(fullfile(agDir, best), 'agent'); agent = S.agent; %#ok<NASGU>
save(fullfile(agDir, 'best_agent_final.mat'), 'agent', 'best');
% trajetoria do melhor: fase em que andou mais (empate -> menor score)
Tb = sortrows(T(T.agent == best, :), {'surv', 'score'}, {'descend', 'ascend'});
finalTag = sprintf('final_p%02d', round(100 * Tb.phase(1)));
for ext = {'_traj.mat', '_motion.mot'}
    copyfile(fullfile(resDir, [char(Tb.tag(1)) ext{1}]), fullfile(resDir, [finalTag ext{1}]));
end
fprintf('melhor: %s | fase %.2f | %.1f s | v=%.2f | score=%.2f | dentro 1DP=%.0f%%\n', best, Tb.phase(1), ...
    Tb.surv(1), Tb.v(1), Tb.score(1), Tb.inBand(1));
if o.render, render_opensim_ds(finalTag); end
end
