function S = gait_cycle_comparison(varargin)
%GAIT_CYCLE_COMPARISON Angulos articulares e forcas de reacao do solo da
%   simulacao, por ciclo de marcha (0-100 %), contra a marcha saudavel do
%   dataset publico de Fukuchi, Fukuchi & Duarte (2018, PeerJ 6:e4640;
%   figshare 10.6084/m9.figshare.5722711, CC BY 4.0): 42 adultos, marcha
%   no solo em velocidade lenta (S, 0,86 +/- 0,13 m/s) e confortavel
%   (C, 1,27 +/- 0,16 m/s).
%
%   Simulacao: cada passada vai de um contato inicial (forca vertical do pe
%   passa de 'thr' N) ao seguinte do mesmo pe; e normalizada em 101 pontos.
%   Saudavel: media de cada sujeito (perna D e E juntas) -> media +/- 1 DP
%   entre sujeitos.
%
%   S = gait_cycle_comparison('tags', {'best_p00','best_p25','best_p50','best_p75'})
%   Gera docs/figs/ciclo_comparacao.csv (para o pgfplots do relatorio) e
%   docs/figs/ciclo_resumo.csv.

p = inputParser;
p.addParameter('tags', {'best_p00','best_p25','best_p50','best_p75'});
p.addParameter('thr', 20);          % N, limiar de contato
p.addParameter('skip', 1.0);        % s iniciais descartados (transitorio)
p.parse(varargin{:}); o = p.Results;
import org.opensim.modeling.*

here = fileparts(mfilename('fullpath'));
root = fullfile(here, '..');
resDir = fullfile(root, 'results'); figDir = fullfile(root, 'docs', 'figs');
dsDir = fullfile(root, '..', 'data', 'wbds', 'proc');
pct = (0:100)';
vars = {'hip','knee','ankle','grfv','grfap'};

%% ---------- marcha saudavel (dataset) ----------
H = struct();
for cls = 'SC'
    fa = dir(fullfile(dsDir, sprintf('WBDS*walkO%cang.txt', cls)));
    X = struct(); for v = vars, X.(v{1}) = []; end
    for i = 1:numel(fa)
        a = readtable(fullfile(fa(i).folder, fa(i).name), 'FileType', 'text', 'Delimiter', '\t');
        k = readtable(fullfile(fa(i).folder, strrep(fa(i).name, 'ang.txt', 'knt.txt')), 'FileType', 'text', 'Delimiter', '\t');
        g = 9.81;    % GRF do dataset em N/kg -> peso corporal (BW)
        X.hip(:,end+1)   = mean([a.RHipAngleZ   a.LHipAngleZ], 2, 'omitnan');
        X.knee(:,end+1)  = mean([a.RKneeAngleZ  a.LKneeAngleZ], 2, 'omitnan');
        X.ankle(:,end+1) = mean([a.RAnkleAngleZ a.LAnkleAngleZ], 2, 'omitnan');
        X.grfv(:,end+1)  = mean([k.RGRFY k.LGRFY], 2, 'omitnan') / g;
        X.grfap(:,end+1) = mean([k.RGRFX k.LGRFX], 2, 'omitnan') / g;
    end
    for v = vars
        Y = X.(v{1}); Y = Y(:, all(isfinite(Y), 1));
        H.(cls).(v{1}).m = mean(Y, 2); H.(cls).(v{1}).s = std(Y, 0, 2); H.(cls).(v{1}).n = size(Y, 2);
    end
end

%% ---------- simulacao: passadas normalizadas ----------
m = Model(gaitConfig().modelFile); st = m.initSystem();
BW = m.getTotalMass(st) * 9.81;
C = struct('R', struct(), 'L', struct());
for side = 'RL', for v = vars, C.(side).(v{1}) = []; end, end
stance = struct('R', [], 'L', []); nStr = struct('R', 0, 'L', 0);
for it = 1:numel(o.tags)
    T = load(fullfile(resDir, [o.tags{it} '_traj.mat'])); tr = T.tr;
    if ~isfield(tr, 'grf3') || isempty(tr.grf3), error('%s sem grf3: reavalie com o ambiente atual', o.tags{it}); end
    t = tr.t; q = rad2deg(tr.q);
    % colunas de allCoords: 7 hip_flexion_r, 10 knee_angle_r, 11 ankle_angle_r, 12 hip_flexion_l, 14 knee_angle_l, 15 ankle_angle_l
    sig.R = struct('hip', q(:,7),  'knee', -q(:,10), 'ankle', q(:,11), 'grfv', tr.grf3(:,2)/BW, 'grfap', tr.grf3(:,1)/BW);
    sig.L = struct('hip', q(:,12), 'knee', -q(:,14), 'ankle', q(:,15), 'grfv', tr.grf3(:,5)/BW, 'grfap', tr.grf3(:,4)/BW);
    for side = 'RL'
        fz = sig.(side).grfv * BW;
        on = find(fz(2:end) >= o.thr & fz(1:end-1) < o.thr) + 1;     % contatos iniciais
        on = on(t(on) >= o.skip);
        for j = 1:numel(on)-1
            i0 = on(j); i1 = on(j+1);
            if t(i1) - t(i0) < 0.5 || t(i1) - t(i0) > 2.0, continue; end   % descarta passadas espurias
            tt = (t(i0:i1) - t(i0)) / (t(i1) - t(i0)) * 100;
            for v = vars
                C.(side).(v{1})(:,end+1) = interp1(tt, sig.(side).(v{1})(i0:i1), pct);
            end
            off = find(fz(i0:i1) < o.thr, 1);                      % fim do apoio
            stance.(side)(end+1) = 100 * (off - 1) / (i1 - i0);
            nStr.(side) = nStr.(side) + 1;
        end
    end
end

%% ---------- tabela para o pgfplots e resumo ----------
% colunas: <grupo>_<variavel>_{m,lo,hi} com lo/hi = media -/+ 1 DP
% grupos: hS/hC = saudavel lento/confortavel; sR/sL = simulacao protese/intacta
cols = {'pct'}; D = pct;
for v = vars
    G = {};   % {nome, media, dp}
    for cls = 'SC', G(end+1,:) = {sprintf('h%c_%s', cls, v{1}), H.(cls).(v{1}).m, H.(cls).(v{1}).s}; end %#ok<AGROW>
    for side = 'RL'
        Y = C.(side).(v{1}); G(end+1,:) = {sprintf('s%c_%s', side, v{1}), mean(Y, 2), std(Y, 0, 2)}; %#ok<AGROW>
    end
    for g = 1:size(G, 1)
        cols = [cols, {[G{g,1} '_m'], [G{g,1} '_lo'], [G{g,1} '_hi']}]; %#ok<AGROW>
        D = [D, G{g,2}, G{g,2} - G{g,3}, G{g,2} + G{g,3}]; %#ok<AGROW>
    end
end
% apoio no saudavel: primeiro ponto apos 40 % com forca vertical < 2 % do peso
for cls = 'SC'
    fa = dir(fullfile(dsDir, sprintf('WBDS*walkO%cknt.txt', cls))); sp = [];
    for i = 1:numel(fa)
        k = readtable(fullfile(fa(i).folder, fa(i).name), 'FileType', 'text', 'Delimiter', '\t');
        for f = {'RGRFY', 'LGRFY'}
            y = k.(f{1}) / 9.81; j = find(pct > 40 & y < 0.02, 1);
            if ~isempty(j), sp(end+1) = pct(j); end %#ok<AGROW>
        end
    end
    H.(cls).stance = [mean(sp), std(sp)];
end
Tt = array2table(D, 'VariableNames', cols);
writetable(Tt, fullfile(figDir, 'ciclo_comparacao.csv'));
% faixas como poligonos fechados (limite superior 0->100 e inferior 100->0),
% para o pgfplots preencher sem costuras
B = table([pct; flipud(pct)], 'VariableNames', {'bx'});
for c = 2:3:numel(cols)
    name = erase(cols{c}, '_m');
    B.(name) = [Tt.([name '_hi']); flipud(Tt.([name '_lo']))];
end
writetable(B, fullfile(figDir, 'ciclo_faixas.csv'));

R = table();
for side = 'RL'
    for v = vars
        ym = mean(C.(side).(v{1}), 2); hm = H.S.(v{1}).m; hs = H.S.(v{1}).s;
        cc = corrcoef(ym, hm);
        R = [R; table(string(side), string(v{1}), sqrt(mean((ym - hm).^2)), cc(1,2), ...
            100*mean(abs(ym - hm) <= hs), 'VariableNames', {'lado','variavel','RMSE','corr','pctDentroDP'})]; %#ok<AGROW>
    end
end
writetable(R, fullfile(figDir, 'ciclo_resumo.csv'));
fprintf('passadas: D=%d  E=%d | sujeitos saudaveis: S=%d  C=%d | massa do modelo=%.1f kg\n', ...
    nStr.R, nStr.L, H.S.hip.n, H.C.hip.n, BW/9.81);
fprintf('apoio (%% do ciclo): protese %.1f +/- %.1f | intacta %.1f +/- %.1f | saudavel lento %.1f +/- %.1f | confortavel %.1f +/- %.1f\n', ...
    mean(stance.R), std(stance.R), mean(stance.L), std(stance.L), H.S.stance, H.C.stance);
disp(R);
S = struct('H', H, 'C', C, 'stance', stance, 'nStrides', nStr, 'summary', R);
save(fullfile(figDir, 'ciclo_comparacao.mat'), 'S');
end
