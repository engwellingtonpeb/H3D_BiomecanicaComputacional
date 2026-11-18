function M = evaluate_4s(agent, varargin)
%EVALUATE_4S Simula a politica (deterministica) da etapa 4 (4 barras, sem
%   dataset) e mede a marcha sem nenhuma referencia externa:
%     survivalTime, meanSpeed, meanReward (o proprio objetivo, por passo),
%     nSteps (passos alternados), apoio (% do ciclo) de cada lado,
%     simetria SI = 100 (D - E)/media de apoio, tempo e comprimento do passo,
%     pico de GRF vertical (PC), esforco medio (sum Fmax_i e_i^2 / sum Fmax).
%
%   M = evaluate_4s(agent, 'duration', 10, 'tag', 'x', 'init', 1)
%   Gera results/<tag>_traj.mat e results/<tag>_motion.mot.
p = inputParser;
p.addParameter('duration', 10); p.addParameter('tag', 'eval'); p.addParameter('quiet', false);
p.addParameter('init', 1);          % indice do estado inicial no banco
p.addParameter('skip', 1.0);
p.parse(varargin{:}); o = p.Results;

here = fileparts(mfilename('fullpath'));
outDir = fullfile(here, '..', 'results'); if ~exist(outDir, 'dir'), mkdir(outDir); end
cfg = gaitConfig4S('maxSteps', round(o.duration / 0.01));
env = FreeGaitEnv(cfg); env.record = true; env.fixedInit = o.init;
obs = reset(env);
if ~isempty(agent), try, agent.UseExplorationPolicy = false; catch, end, end
for k = 1:cfg.maxSteps
    if isempty(agent), a = zeros(15,1); else, a = getAction(agent, {obs}); a = a{1}; end
    [obs, ~, done] = step(env, a);
    if done, break; end
end
tr = env.traj;
tr.BW = modelWeight(cfg.modelFile);
M = gaitStats(tr, cfg, o.skip);
M.tag = o.tag; M.init = o.init;
M.survivalTime = tr.t(end);
M.meanSpeed = (tr.q(end,4) - tr.q(1,4)) / max(tr.t(end) - tr.t(1), eps);
M.meanReward = mean(tr.r);
M.effort = mean(tr.e.^2 * (cfg.Fmax(:) / sum(cfg.Fmax)));
writeMot(fullfile(outDir, [o.tag '_motion.mot']), tr, cfg);
save(fullfile(outDir, [o.tag '_traj.mat']), 'tr', 'M', 'cfg');
if ~o.quiet
    fprintf('%s | %.1f s | v=%.2f m/s | r/passo=%.3f | passos=%d | apoio D/E=%.0f/%.0f%% | SI passo t/c=%.0f/%.0f%% | esforco=%.3f\n', ...
        o.tag, M.survivalTime, M.meanSpeed, M.meanReward, M.nSteps, M.stance.R, M.stance.L, M.SI.stepTime, M.SI.stepLen, M.effort);
end
end

function M = gaitStats(tr, cfg, skip)
thr = cfg.grfThr; t = tr.t; BW = tr.BW; side = 'RL';
for i = 1:2
    fz = tr.grf3(:, 3*i-1);
    on = find(fz(2:end) >= thr & fz(1:end-1) < thr) + 1; on = on(t(on) >= skip);
    st = []; pk = [];
    for j = 1:numel(on)-1
        i0 = on(j); i1 = on(j+1); T = t(i1) - t(i0); if T < 0.4 || T > 2.5, continue, end
        off = find(fz(i0:i1) < thr, 1); if isempty(off), continue, end
        st(end+1) = 100 * (off - 1) / (i1 - i0); pk(end+1) = max(fz(i0:i1)) / BW; %#ok<AGROW>
    end
    M.stance.(side(i)) = mean(st); M.peakV.(side(i)) = mean(pk); M.nStrides.(side(i)) = numel(st);
end
S = tr.steps; S = S(S(:,2) >= skip, :);
M.nSteps = size(S, 1);
si = @(a, b) 100 * (a - b) / (0.5 * (a + b));
tR = mean(S(S(:,1)==1, 4)); tL = mean(S(S(:,1)==2, 4));
lR = mean(S(S(:,1)==1, 3)); lL = mean(S(S(:,1)==2, 3));
M.stepTime = [tR tL]; M.stepLen = [lR lL];
M.SI.stepTime = si(tR, tL); M.SI.stepLen = si(lR, lL);
M.SI.stance = si(M.stance.R, M.stance.L); M.SI.peakV = si(M.peakV.R, M.peakV.L);
end

function W = modelWeight(file)
persistent cache
if isempty(cache)
    m = org.opensim.modeling.Model(file); s = m.initSystem(); cache = m.getTotalMass(s) * 9.81;
end
W = cache;
end

function writeMot(file, tr, cfg)
names = [cfg.allCoords cfg.fourBar.depCoords]; rot = ~ismember(names, {'pelvis_tx','pelvis_ty','pelvis_tz'});
D = [tr.qraw tr.dep]; D(:,rot) = rad2deg(D(:,rot));
fid = fopen(file, 'w');
fprintf(fid, 'gait_rl_etapa4\nversion=1\nnRows=%d\nnColumns=%d\ninDegrees=yes\nendheader\n', size(D,1), size(D,2)+1);
fprintf(fid, '%s', 'time'); fprintf(fid, '\t%s', names{:}); fprintf(fid, '\n');
fprintf(fid, [repmat('%.6f\t', 1, size(D,2)) '%.6f\n'], [tr.t D]');
fclose(fid);
end
