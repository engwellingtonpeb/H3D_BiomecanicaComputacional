function M = evaluate_ds(agent, varargin)
%EVALUATE_DS Simula a politica (deterministica) da etapa 2 e compara cada
%   passada com a marcha saudavel do dataset (media +/- 1 DP).
%
%   M = evaluate_ds(agent, 'baseWeight', 0, 'duration', 10, 'tag', 'x')
%   Gera results/<tag>_traj.mat e results/<tag>_motion.mot.
%
%   Metricas por passada (contato inicial = GRF vertical cruza 20 N),
%   normalizada em 101 pontos e comparada com o dataset:
%     z.<var>.<lado>   erro RMS em unidades de DP entre sujeitos
%     in.<var>.<lado>  % dos pontos do ciclo dentro de +/- 1 DP
%     rmse/corr        em graus (angulos) ou BW (GRF)
%   score = media dos z (angulos + GRF, dois lados): menor e melhor.
p = inputParser;
p.addParameter('baseWeight', 0); p.addParameter('duration', 10);
p.addParameter('tag', 'eval'); p.addParameter('quiet', false);
p.addParameter('initPhase', 0); p.addParameter('skip', 1.0);
p.parse(varargin{:}); o = p.Results;

here = fileparts(mfilename('fullpath'));
outDir = fullfile(here, '..', 'results'); if ~exist(outDir, 'dir'), mkdir(outDir); end
cfg = gaitConfigDS('baseWeight', o.baseWeight, 'randomInitPhase', false, 'initPhase', o.initPhase, ...
                   'maxSteps', round(o.duration / 0.01));
env = DatasetGaitEnv(cfg); env.record = true;
obs = reset(env);
if ~isempty(agent), try, agent.UseExplorationPolicy = false; catch, end, end
for k = 1:cfg.maxSteps
    if isempty(agent), a = zeros(15,1); else, a = getAction(agent, {obs}); a = a{1}; end
    [obs, ~, done] = step(env, a);
    if done, break; end
end
tr = env.traj; ref = env.ref;
M = strideMetrics(tr, ref, env.cfg, o.skip);
M.tag = o.tag; M.baseWeight = o.baseWeight;
M.survivalTime = tr.t(end);
M.meanSpeed = (tr.q(end,4) - tr.q(1,4)) / max(tr.t(end) - tr.t(1), eps);
M.meanExcitation = mean(tr.e, 'all');
M.excitationChange = mean(abs(diff(tr.e)), 'all');
writeMot(fullfile(outDir, [o.tag '_motion.mot']), tr, cfg);
save(fullfile(outDir, [o.tag '_traj.mat']), 'tr', 'M', 'cfg');
if ~o.quiet
    fprintf('%s | %.1f s | v=%.2f m/s | passadas D/E=%d/%d | score(z)=%.2f | dentro de 1DP=%.0f%%\n', ...
        o.tag, M.survivalTime, M.meanSpeed, M.nStrides.R, M.nStrides.L, M.score, M.inBand);
end
end

function M = strideMetrics(tr, ref, cfg, skip)
thr = 20; pct = (0:100)'/100;
BW = 67.8 * 9.81;     % massa total do modelo
q = rad2deg(tr.q); t = tr.t;
% colunas de allCoords: 7 hip_r, 10 knee_r, 11 ankle_r, 12 hip_l, 14 knee_l, 15 ankle_l
sig.R = struct('hip', q(:,7),  'knee', q(:,10), 'ankle', q(:,11), 'grfv', tr.grf3(:,2)/BW, 'grfap', tr.grf3(:,1)/BW);
sig.L = struct('hip', q(:,12), 'knee', q(:,14), 'ankle', q(:,15), 'grfv', tr.grf3(:,5)/BW, 'grfap', tr.grf3(:,4)/BW);
vars = {'hip','knee','ankle','grfv','grfap'};
unit = struct('hip', 180/pi, 'knee', 180/pi, 'ankle', 180/pi, 'grfv', 1, 'grfap', 1);
zs = []; ins = [];
for side = 'RL'
    fz = sig.(side).grfv * BW;
    on = find(fz(2:end) >= thr & fz(1:end-1) < thr) + 1;
    on = on(t(on) >= skip);
    C = struct(); for v = vars, C.(v{1}) = []; end; st = [];
    for j = 1:numel(on)-1
        i0 = on(j); i1 = on(j+1); T = t(i1) - t(i0);
        if T < 0.5 || T > 2.0, continue; end
        tt = (t(i0:i1) - t(i0)) / T;
        for v = vars, C.(v{1})(:,end+1) = interp1(tt, sig.(side).(v{1})(i0:i1), pct); end
        off = find(fz(i0:i1) < thr, 1); st(end+1) = 100 * (off - 1) / (i1 - i0); %#ok<AGROW>
    end
    M.nStrides.(side) = size(C.hip, 2);
    M.stance.(side) = mean(st);
    for v = vars
        hm = unit.(v{1}) * interp1([ref.phase; 1], [ref.(v{1}); ref.(v{1})(1)], pct);
        hs = unit.(v{1}) * interp1([ref.phase; 1], [ref.([v{1} 'SD']); ref.([v{1} 'SD'])(1)], pct);
        if isempty(C.(v{1}))
            M.z.(v{1}).(side) = NaN; M.in.(v{1}).(side) = 0; M.rmse.(v{1}).(side) = NaN; M.corr.(v{1}).(side) = NaN;
            zs(end+1) = 10; ins(end+1) = 0; continue %#ok<AGROW>
        end
        ym = mean(C.(v{1}), 2);
        M.z.(v{1}).(side) = sqrt(mean(((ym - hm) ./ hs).^2));
        M.in.(v{1}).(side) = 100 * mean(abs(ym - hm) <= hs);
        M.rmse.(v{1}).(side) = sqrt(mean((ym - hm).^2));
        cc = corrcoef(ym, hm); M.corr.(v{1}).(side) = cc(1,2);
        M.cycle.(side).(v{1}) = ym;
        zs(end+1) = M.z.(v{1}).(side); ins(end+1) = M.in.(v{1}).(side); %#ok<AGROW>
    end
end
M.score = mean(min(zs, 10));
M.inBand = mean(ins);
M.refStance = 100 * ref.stanceFrac;
end

function writeMot(file, tr, cfg)
names = cfg.allCoords; rot = ~ismember(names, {'pelvis_tx','pelvis_ty','pelvis_tz'});
D = tr.q; D(:,rot) = rad2deg(D(:,rot));
fid = fopen(file, 'w');
fprintf(fid, 'gait_rl_etapa2\nversion=1\nnRows=%d\nnColumns=%d\ninDegrees=yes\nendheader\n', size(D,1), size(D,2)+1);
fprintf(fid, '%s', 'time'); fprintf(fid, '\t%s', names{:}); fprintf(fid, '\n');
fprintf(fid, [repmat('%.6f\t', 1, size(D,2)) '%.6f\n'], [tr.t D]');
fclose(fid);
end
