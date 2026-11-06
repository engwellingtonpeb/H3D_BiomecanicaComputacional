function ref = datasetReference(varargin)
%DATASETREFERENCE Referencia de marcha saudavel construida a partir do
%   dataset publico de Fukuchi, Fukuchi & Duarte (2018, PeerJ 6:e4640;
%   figshare 10.6084/m9.figshare.5722711, CC BY 4.0), marcha no solo.
%
%   ref = datasetReference('class','S')   % 'S' lenta (0,86 m/s), 'C' confortavel
%
%   Para cada variavel: media e DP entre sujeitos (media das pernas D e E
%   de cada sujeito) em 1000 pontos de fase (0 = contato inicial):
%     hip, knee, ankle (rad, convencao OpenSim: knee<0 = flexao), hipAdd
%     grfv, grfap (peso corporal, forca SOBRE o pe, AP + para frente)
%   Mais: velocidade media, duracao da passada (marcadores do calcanhar) e
%   % de apoio. O resultado fica em cache em v2_dataset/data/ref_<class>.mat.

p = inputParser;
p.addParameter('class', 'S');
p.addParameter('rebuild', false);
p.parse(varargin{:}); o = p.Results;

here = fileparts(mfilename('fullpath'));
root = fullfile(here, '..', '..');
cacheFile = fullfile(here, '..', 'data', sprintf('ref_%s.mat', o.class));
if isfile(cacheFile) && ~o.rebuild
    S = load(cacheFile); ref = S.ref; return
end
dsDir = fullfile(root, 'data', 'wbds', 'proc');
mkDir = fullfile(root, 'data', 'wbds', 'mkr');
g = 9.81;

% ---------------- curvas (101 pontos) ----------------
fa = dir(fullfile(dsDir, sprintf('WBDS*walkO%cang.txt', o.class)));
X = struct('hip',[],'knee',[],'ankle',[],'hipAdd',[],'grfv',[],'grfap',[]);
for i = 1:numel(fa)
    a = readtable(fullfile(fa(i).folder, fa(i).name), 'FileType', 'text', 'Delimiter', '\t');
    k = readtable(fullfile(fa(i).folder, strrep(fa(i).name, 'ang.txt', 'knt.txt')), 'FileType', 'text', 'Delimiter', '\t');
    X.hip(:,end+1)    = mean([a.RHipAngleZ   a.LHipAngleZ], 2, 'omitnan');
    X.knee(:,end+1)   = mean([a.RKneeAngleZ  a.LKneeAngleZ], 2, 'omitnan');
    X.ankle(:,end+1)  = mean([a.RAnkleAngleZ a.LAnkleAngleZ], 2, 'omitnan');
    X.hipAdd(:,end+1) = mean([a.RHipAngleX a.LHipAngleX], 2, 'omitnan');   % X = aducao + (ja espelhado no Visual3D)
    X.grfv(:,end+1)   = mean([k.RGRFY k.LGRFY], 2, 'omitnan') / g;
    X.grfap(:,end+1)  = mean([k.RGRFX k.LGRFX], 2, 'omitnan') / g;
end
N = 1000; ph = (0:N-1)'/N; pct = (0:100)'/100;
sgn = struct('hip', pi/180, 'knee', -pi/180, 'ankle', pi/180, 'hipAdd', pi/180, 'grfv', 1, 'grfap', 1);
for v = fieldnames(X)'
    Y = X.(v{1}); Y = Y(:, all(isfinite(Y), 1));
    m = mean(Y, 2); s = std(Y, 0, 2);
    % ciclo periodico: 0 e 100 % sao o mesmo instante -> usa 0..99 %
    ref.(v{1})       = sgn.(v{1}) * interp1([pct(1:100); 1], [m(1:100); m(1)], ph, 'pchip');
    ref.([v{1} 'SD']) = abs(sgn.(v{1})) * interp1([pct(1:100); 1], [s(1:100); s(1)], ph, 'pchip');
    ref.nSubjects = size(Y, 2);
end
% hipAdd: Visual3D usa X + = aducao na perna D; confere pelo pico no inicio do apoio
if mean(ref.hipAdd(1:200)) < 0, ref.hipAdd = -ref.hipAdd; end

% ---------------- velocidade e duracao da passada ----------------
info = readtable(fullfile(root, 'data', 'wbds', 'WBDSinfo.csv'), 'Delimiter', ',', 'TextType', 'string');
sel = endsWith(info.FileName, sprintf('%cmkr.txt', o.class)) & contains(info.FileName, 'walkO');
ref.speed = mean(double(info.GaitSpeed_m_s_(sel)), 'omitnan');
ref.cycleTime = strideTime(mkDir, o.class);

% % de apoio: primeiro ponto apos 40 % com forca vertical < 2 % do peso
j = find(ph > 0.4 & ref.grfv < 0.02, 1); ref.stanceFrac = ph(j);
ref.phase = ph; ref.N = N; ref.class = o.class;
ref.source = 'Fukuchi, Fukuchi & Duarte (2018) PeerJ 6:e4640, CC BY 4.0';
save(cacheFile, 'ref');
end

function T = strideTime(mkDir, cls)
% duracao da passada pelo calcanhar: inicio de cada periodo parado (velocidade
% horizontal < 0,25 m/s por >= 60 ms) ate o seguinte do mesmo pe
f = dir(fullfile(mkDir, sprintf('WBDS*walkO*%cmkr.txt', cls)));
bySubj = containers.Map('KeyType', 'char', 'ValueType', 'any');
for i = 1:numel(f)
    M = readmatrix(fullfile(f(i).folder, f(i).name), 'FileType', 'text', 'NumHeaderLines', 1);
    hdr = strsplit(fgetl(fopen(fullfile(f(i).folder, f(i).name))), '\t'); fclose('all');
    t = M(:,1); dt = median(diff(t));
    for side = ["R", "L"]
        cx = find(strcmp(hdr, side + ".HeelX")); cz = find(strcmp(hdr, side + ".HeelZ"));
        if isempty(cx), continue; end
        P = M(:, [cx cz]) / 1000;
        if any(~isfinite(P(:))), P = fillmissing(P, 'linear'); end
        v = [0; sqrt(sum(diff(P).^2, 2)) / dt]; v = movmean(v, 5);
        st = v < 0.25; d = diff([0; st; 0]); on = find(d == 1); off = find(d == -1) - 1;
        keep = (off - on) * dt >= 0.06; on = on(keep);
        s = diff(t(on)); s = s(s > 0.7 & s < 2.0);
        key = f(i).name(1:6);
        if ~isKey(bySubj, key), bySubj(key) = []; end
        bySubj(key) = [bySubj(key); s(:)];
    end
end
vals = cellfun(@(x) mean(x), values(bySubj)); vals = vals(isfinite(vals));
T = mean(vals);
fprintf('duracao da passada (%c): %.3f +/- %.3f s (%d sujeitos)\n', cls, T, std(vals), numel(vals));
end
