function R = compare_etapas(varargin)
%COMPARE_ETAPAS Etapa 4: compara CIR x fase x GRF e assimetria entre
%   - etapa 2: protese de pino, referencia do dataset
%   - etapa 3: 4 barras, referencia do dataset
%   - etapa 4: 4 barras, sem dataset (so plausibilidade fisiologica)
%   e a marcha saudavel do dataset (so como contexto; as curvas medias
%   vem de etapa3_FD_prothesis_4barras/results/cir_asym_dados.mat).
%
%   R = compare_etapas('traj3', <final da etapa 3>, 'traj4', <final da etapa 4>)
%   Saidas (etapa 4): results/comparacao_dados.mat, results/comparacao_assimetria.csv,
%     results/comparacao_cir_fase_grf.csv, figs/cmp_cir_trajetoria.png,
%     figs/cmp_cir_fase_grf.png, figs/cmp_assimetria.png
here = fileparts(mfilename('fullpath')); root = fullfile(here, '..', '..');
p = inputParser;
p.addParameter('traj2', fullfile(root, 'etapa2_FD_prothesis_Dataset', 'results', 'final_p50_traj.mat'));
p.addParameter('traj3', '');
p.addParameter('traj4', fullfile(here, '..', 'results', 'final_i0250_traj.mat'));
p.addParameter('plot', true);
p.parse(varargin{:}); o = p.Results;
if isempty(o.traj3)
    f = dir(fullfile(root, 'etapa3_FD_prothesis_4barras', 'results', 'final_p*_traj.mat'));
    o.traj3 = fullfile(f(1).folder, f(1).name);
end
resDir = fullfile(here, '..', 'results'); figDir = fullfile(here, '..', 'figs');
F = fourBarMap(gaitConfig4S());

R.pin = gaitAnalysis(o.traj2, 'pin');
R.fb3 = gaitAnalysis(o.traj3, 'fourbar');
R.fb4 = gaitAnalysis(o.traj4, 'fourbar');
src = {'pin', 'fb3', 'fb4'}; files = {o.traj2, o.traj3, o.traj4};

D = load(fullfile(root, 'etapa3_FD_prothesis_4barras', 'results', 'cir_asym_dados.mat'), ...
    'pct', 'ref_knee', 'ref_kneeSD', 'ref_grfV', 'ref_grfVSD', 'ref_grfAP', 'ref_grfAPSD', 'ref_stance');
ok = all(isfinite(F.cir), 2);
D.cent_flex = rad2deg(F.flex(ok)); D.cent_cir = 1000 * F.cir(ok,:); D.piv0 = 1000 * F.piv(1,:);
D.pin = 1000 * R.pin.pinCenter;
pct = (0:100)';
vars = {'kneeFlex', 'cirX', 'cirY', 'grfV', 'grfAP', 'dcir', 'Mcir'};
T = table(pct, 'VariableNames', {'pct_ciclo_protese'});
Ta = R.pin.params(:, {'param'});
for k = 1:3
    A = R.(src{k});
    for v = vars
        [D.([src{k} '_' v{1}]), D.([src{k} '_' v{1} 'SD'])] = cycStat(A.cyc.(v{1}));
        T.([src{k} '_' v{1}]) = D.([src{k} '_' v{1}]);
    end
    D.([src{k} '_stanceEnd']) = 100 * mean(A.stanceEnd);
    D.([src{k} '_nStrides']) = A.nStrides;
    S = load(files{k}, 'tr'); D.([src{k} '_speed']) = (S.tr.q(end,4) - S.tr.q(1,4)) / (S.tr.t(end) - S.tr.t(1));
    Ta.([src{k} '_D']) = A.params.protese_D; Ta.([src{k} '_E']) = A.params.intacta_E; Ta.([src{k} '_SI']) = A.params.SI_pct;
    D.(['asym_' src{k}]) = A.params.SI_pct;
    if k > 1
        okp = isfinite(A.phase.R);
        D.([src{k} '_cirT']) = 1000 * A.cirT(okp,:); D.([src{k} '_phase']) = 100 * A.phase.R(okp);
    end
end
D.asym_param = cellstr(Ta.param);
D.files = files;
writetable(T, fullfile(resDir, 'comparacao_cir_fase_grf.csv'));
writetable(Ta, fullfile(resDir, 'comparacao_assimetria.csv'));
datFile = fullfile(resDir, 'comparacao_dados.mat');
save(datFile, '-struct', 'D');
R.table = Ta; R.curves = T;
disp(Ta);
fprintf('velocidade media (m/s): pino %.2f | 4 barras c/ dataset %.2f | 4 barras s/ dataset %.2f\n', D.pin_speed, D.fb3_speed, D.fb4_speed);
fprintf('passadas analisadas [D E]: pino %s | e3 %s | e4 %s\n', mat2str(D.pin_nStrides), mat2str(D.fb3_nStrides), mat2str(D.fb4_nStrides));

if o.plot
    py = fullfile(here, 'plot_comparacao.py');
    [st, msg] = system(sprintf('python "%s" "%s" "%s"', py, datFile, figDir));
    if st ~= 0, warning('figuras: %s', msg); else, fprintf('%s', msg); end
end
end

function [m, s] = cycStat(C)
m = mean(C, 2, 'omitnan'); s = std(C, 0, 2, 'omitnan'); s(~isfinite(s)) = 0;
end
