function M = evaluate_policy(agent, varargin)
%EVALUATE_POLICY Simula a politica (deterministica) e mede a qualidade da marcha.
%   M = evaluate_policy(agent, 'assist', 0, 'duration', 10, 'tag', 'final')
%   agent = [] -> apenas o controlador-base (acao zero).
%   Gera: results/<tag>_motion.mot (abrir no OpenSim GUI sobre o modelo),
%         results/<tag>_gait.png e retorna metricas.
p = inputParser;
p.addParameter('assist', 0); p.addParameter('duration', 10);
p.addParameter('tag', 'eval'); p.addParameter('quiet', false);
p.addParameter('initPhase', 0);
p.parse(varargin{:}); o = p.Results;

here = fileparts(mfilename('fullpath'));
outDir = fullfile(here, '..', 'results'); if ~exist(outDir, 'dir'), mkdir(outDir); end
cfg = gaitConfig('assistStart', o.assist, 'randomInitPhase', false, 'initPhase', o.initPhase, ...
                 'maxSteps', round(o.duration / 0.01));
env = ProstheticGaitEnv(cfg); env.record = true;
obs = reset(env);
if ~isempty(agent)
    try, agent.UseExplorationPolicy = false; catch, end
end
for k = 1:cfg.maxSteps
    if isempty(agent), a = zeros(15,1); else, a = getAction(agent, {obs}); a = a{1}; end
    [obs, ~, done] = step(env, a);
    if done, break; end
end
tr = env.traj; M = gaitMetrics(tr, cfg, env.ref);
M.tag = o.tag;
writeMot(fullfile(outDir, [o.tag '_motion.mot']), tr, cfg);
save(fullfile(outDir, [o.tag '_traj.mat']), 'tr', 'M', 'cfg');
% graficos so no MATLAB desktop (em -batch os graficos derrubam o MATLAB nesta maquina)
if usejava('desktop'), plotGait(fullfile(outDir, [o.tag '_gait.png']), tr, cfg, env.ref, M); end
writematrix([tr.t tr.q tr.grf tr.e], fullfile(outDir, [o.tag '_traj.csv']));
if ~o.quiet, disp(M); end
end

function M = gaitMetrics(tr, cfg, ref)
t = tr.t; q = tr.q; n = numel(t);
M.survivalTime = t(end);
M.meanSpeed = (q(end,4) - q(1,4)) / max(t(end) - t(1), eps);
% fase da referencia em cada amostra (perna esquerda)
ph = mod(cfg.initPhase + t / ref.cycleTime, 1);
idx = [7 10 11 12 14 15];   % hipR kneeR ankR hipL kneeL ankL
sig = {ref.hip, ref.knee, ref.ankle, ref.hip, ref.knee, ref.ankle};
off = [0.5 0.5 0.5 0 0 0];
names = {'hipR','kneeR','ankR','hipL','kneeL','ankL'};
for j = 1:6
    r = interp1((0:ref.N)'/ref.N, [sig{j}; sig{j}(1)], mod(ph + off(j), 1));
    e = rad2deg(q(:,idx(j)) - r);
    M.rmse.(names{j}) = sqrt(mean(e.^2));
    if n > 5, cc = corrcoef(q(:,idx(j)), r); M.corr.(names{j}) = cc(1,2); else, M.corr.(names{j}) = NaN; end
end
M.rmseMean = mean(cell2mat(struct2cell(M.rmse)));
M.corrMean = mean(cell2mat(struct2cell(M.corr)));
% suavidade: jerk medio normalizado das juntas (rad/s^3 normalizado pela amplitude)
dt = cfg.dt;
if n > 10
    J = diff(q(:,idx), 3) / dt^3;
    amp = max(max(q(:,idx)) - min(q(:,idx)), 1e-3);
    M.jerkIndex = mean(sqrt(mean(J.^2)) ./ amp) * (ref.cycleTime^3) / 1e3;
else
    M.jerkIndex = NaN;
end
% simetria do tempo de apoio (pela GRF vertical)
st = tr.grf > 50;
M.stanceR = mean(st(:,1)); M.stanceL = mean(st(:,2));
M.stanceSymmetry = 100 * abs(M.stanceR - M.stanceL) / max(0.5*(M.stanceR + M.stanceL), eps);
M.peakGRF_BW = max(tr.grf) / (9.80665 * 75);
M.excitationChange = mean(abs(diff(tr.e(:,1:15))), 'all');
M.meanExcitation = mean(tr.e(:,1:15), 'all');
M.plausible = M.survivalTime >= 0.98*cfg.maxSteps*cfg.dt && M.meanSpeed > 0.9 && M.meanSpeed < 1.5 ...
    && M.rmseMean < 10 && M.corrMean > 0.8 && M.stanceSymmetry < 15;
end

function writeMot(file, tr, cfg)
names = cfg.allCoords; rot = ~ismember(names, {'pelvis_tx','pelvis_ty','pelvis_tz'});
D = tr.q; D(:,rot) = rad2deg(D(:,rot));
fid = fopen(file, 'w');
fprintf(fid, 'gait_rl\nversion=1\nnRows=%d\nnColumns=%d\ninDegrees=yes\nendheader\n', size(D,1), size(D,2)+1);
fprintf(fid, '%s', 'time'); fprintf(fid, '\t%s', names{:}); fprintf(fid, '\n');
fprintf(fid, [repmat('%.6f\t', 1, size(D,2)) '%.6f\n'], [tr.t D]');
fclose(fid);
end

function plotGait(file, tr, cfg, ref, M)
f = figure('Visible', 'off', 'Position', [100 100 1200 800]);
ph = mod(cfg.initPhase + tr.t / ref.cycleTime, 1);
idx = [7 10 11 12 14 15]; sig = {ref.hip, ref.knee, ref.ankle, ref.hip, ref.knee, ref.ankle};
off = [0.5 0.5 0.5 0 0 0]; ttl = {'Quadril D (residual)','Joelho D (protese)','Tornozelo D (protese)','Quadril E','Joelho E','Tornozelo E'};
for j = 1:6
    subplot(3,3,j);
    r = interp1((0:ref.N)'/ref.N, [sig{j}; sig{j}(1)], mod(ph + off(j), 1));
    plot(tr.t, rad2deg(r), 'k--', tr.t, rad2deg(tr.q(:,idx(j))), 'b', 'LineWidth', 1.2);
    title(ttl{j}); ylabel('graus'); grid on;
end
subplot(3,3,7); plot(tr.t, tr.grf); title('GRF vertical (N)'); legend('D','E'); grid on;
subplot(3,3,8); plot(tr.t, tr.e(:,1:15)); title('Excitacoes musculares'); ylim([0 1]); grid on;
subplot(3,3,9); plot(tr.t, tr.q(:,5), tr.t, tr.q(:,1)); title('Pelve: altura (m) e inclinacao (rad)'); grid on;
sgtitle(sprintf('%s | %.1f s | v=%.2f m/s | RMSE=%.1f graus | corr=%.2f | simetria=%.0f%%', ...
    strrep(M.tag,'_',' '), M.survivalTime, M.meanSpeed, M.rmseMean, M.corrMean, M.stanceSymmetry));
exportgraphics(f, file); close(f);
end
