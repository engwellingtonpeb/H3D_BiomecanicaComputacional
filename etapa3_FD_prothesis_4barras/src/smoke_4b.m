% teste rapido do ambiente da etapa 3
here = fileparts(mfilename('fullpath')); addpath(here); cd(here);
S = load(fullfile(here,'..','..','etapa2_FD_prothesis_Dataset','agents','best_agent_final.mat'));
for b = [1 0]
    tic; M = evaluate_4b(S.agent, 'baseWeight', b, 'duration', 3, 'tag', sprintf('smoke_b%d', b), 'skip', 0.5);
    T = load(fullfile(here,'..','results',sprintf('smoke_b%d_traj.mat', b))); tr = T.tr;
    fprintf('beta=%d: %.2f s simulados em %.0f s | flexao R min=%.1f deg | elo min=%.1f deg\n', b, M.survivalTime, toc, min(rad2deg(tr.q(:,10))), min(rad2deg(tr.link)));
    i = find(tr.grf3(:,2) > 100, 1); 
    if ~isempty(i), fprintf('   COP R x=%.3f | pe R (tornozelo) ~ x=%.3f | CIR G x=%.3f y=%.3f\n', tr.copR(i,1), tr.kneeG(i,1), tr.cirG(i,1), tr.cirG(i,2)); end
    fprintf('   CIR coxa: x %.1f..%.1f mm, y %.1f..%.1f mm\n', 1000*min(tr.cir(:,1)), 1000*max(tr.cir(:,1)), 1000*min(tr.cir(:,2)), 1000*max(tr.cir(:,2)));
end
