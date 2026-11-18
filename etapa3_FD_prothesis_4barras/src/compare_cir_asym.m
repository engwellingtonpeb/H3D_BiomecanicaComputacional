function R = compare_cir_asym(varargin)
%COMPARE_CIR_ASYM Etapa 3: protese de 4 barras x protese monocentrica (pino,
%   etapa 2) x marcha saudavel do dataset.
%
%   Calcula (gaitAnalysis) e grava os dados; as figuras sao desenhadas em
%   Python/matplotlib (plot_cir_asym.py), que nao depende da placa de video
%   (o MATLAB em -batch trava ao desenhar com a sessao do Windows bloqueada).
%
%   1) trajetoria do CIR da perna protetica em relacao a coxa, colorida pela
%      fase do ciclo, sobre a centroide completa do mecanismo
%   2) curvas sincronizadas no ciclo da protese (0% = contato inicial D):
%      flexao do joelho, CIR (x, y), GRF vertical e antero-posterior e braco
%      de alavanca da GRF em torno do CIR (> 0 = extensor/estavel)
%   3) assimetria (SI) das duas proteses
%
%   R = compare_cir_asym('tag4b', 'final_p00')
%   Saidas: results/cir_asym_dados.mat, results/cir_fase_grf.csv,
%           results/assimetria.csv, figs/cir_trajetoria.png,
%           figs/cir_fase_grf.png, figs/assimetria.png
here = fileparts(mfilename('fullpath'));
p = inputParser;
p.addParameter('tag4b', '');
p.addParameter('traj2', fullfile(here, '..', '..', 'etapa2_FD_prothesis_Dataset', 'results', 'final_p50_traj.mat'));
p.addParameter('plot', true);
p.parse(varargin{:}); o = p.Results;
resDir = fullfile(here, '..', 'results');
cfg = gaitConfig4B(); F = fourBarMap(cfg); ref = datasetReference('class', cfg.refClass);

R.pin = gaitAnalysis(o.traj2, 'pin');
has4 = ~isempty(o.tag4b);
if has4, R.fb = gaitAnalysis(fullfile(resDir, [o.tag4b '_traj.mat']), 'fourbar'); end

% ---------- dados para as figuras ----------
D = struct();
D.has4 = double(has4);
ok = all(isfinite(F.cir), 2);
D.cent_flex = rad2deg(F.flex(ok)); D.cent_cir = 1000 * F.cir(ok,:); D.piv0 = 1000 * F.piv(1,:);
D.pin = 1000 * R.pin.pinCenter;
pct = (0:100)'; D.pct = pct;
iref = @(x) interp1([ref.phase; 1], [x; x(1)], pct/100);
D.ref_knee = iref(ref.knee) * 180/pi; D.ref_kneeSD = iref(ref.kneeSD) * 180/pi;
D.ref_grfV = iref(ref.grfv); D.ref_grfVSD = iref(ref.grfvSD);
D.ref_grfAP = iref(ref.grfap); D.ref_grfAPSD = iref(ref.grfapSD);
D.ref_stance = 100 * ref.stanceFrac;
vars = {'kneeFlex', 'cirX', 'cirY', 'grfV', 'grfAP', 'dcir', 'Mcir'};
T = table(pct, 'VariableNames', {'pct_ciclo_protese'});
for v = vars
    [D.(['pin_' v{1}]), D.(['pin_' v{1} 'SD'])] = cycStat(R.pin.cyc.(v{1}));
    T.(['pino_' v{1}]) = D.(['pin_' v{1}]);
    if has4
        [D.(['fb_' v{1}]), D.(['fb_' v{1} 'SD'])] = cycStat(R.fb.cyc.(v{1}));
        T.(['quatroBarras_' v{1}]) = D.(['fb_' v{1}]);
    end
end
D.pin_stanceEnd = 100 * mean(R.pin.stanceEnd); D.pin_nStrides = R.pin.nStrides;
if has4
    A = R.fb; okp = isfinite(A.phase.R);
    D.fb_cirT = 1000 * A.cirT(okp,:); D.fb_phase = 100 * A.phase.R(okp);
    D.fb_stanceEnd = 100 * mean(A.stanceEnd); D.fb_nStrides = A.nStrides;
    D.fb_tag = o.tag4b;
end
writetable(T, fullfile(resDir, 'cir_fase_grf.csv'));

Ta = R.pin.params(:, {'param'}); Ta.pino_D = R.pin.params.protese_D; Ta.pino_E = R.pin.params.intacta_E; Ta.pino_SI = R.pin.params.SI_pct;
if has4
    Ta.quatroBarras_D = R.fb.params.protese_D; Ta.quatroBarras_E = R.fb.params.intacta_E; Ta.quatroBarras_SI = R.fb.params.SI_pct;
end
writetable(Ta, fullfile(resDir, 'assimetria.csv'));
D.asym_param = cellstr(Ta.param); D.asym_pin = Ta.pino_SI;
if has4, D.asym_fb = Ta.quatroBarras_SI; end
datFile = fullfile(resDir, 'cir_asym_dados.mat');
save(datFile, '-struct', 'D');
R.table = Ta; R.curves = T;

if o.plot
    py = fullfile(here, 'plot_cir_asym.py');
    [st, msg] = system(sprintf('python "%s" "%s" "%s"', py, datFile, fullfile(here, '..', 'figs')));
    if st ~= 0, warning('figuras: %s', msg); else, fprintf('%s', msg); end
end
end

function [m, s] = cycStat(C)
m = mean(C, 2, 'omitnan'); s = std(C, 0, 2, 'omitnan'); s(~isfinite(s)) = 0;
end
