% teste rapido do ambiente da etapa 4: agente da etapa 3 (beta = 0) com as
% entradas do dataset zeradas, com e sem relogio interno
here = fileparts(mfilename('fullpath')); addpath(here); cd(here);
if ~isfile(fullfile(here, '..', 'data', 'init_states.mat')), build_init_states(); end
S = load(fullfile(here, '..', 'data', 'init_agent_etapa3_chunk116.mat'));
for ini = [1 300 700]
    tic; M = evaluate_4s(S.agent, 'duration', 4, 'tag', sprintf('smoke_i%d', ini), 'init', ini, 'skip', 0.3);
    fprintf('   (%.0f s de CPU)\n', toc);
end
T = load(fullfile(here, '..', 'results', 'smoke_i1_traj.mat')); tr = T.tr;
fprintf('termos medios [speed upright step effort smooth rom impact flight vx]:\n'); disp(mean(tr.terms, 1));
disp(tr.steps);
