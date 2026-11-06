function make_media(tag, varargin)
%MAKE_MEDIA Figura-resumo (PNG) e video (MP4) de uma avaliacao salva em
%   results/<tag>_traj.mat.
%   Funciona em "matlab -batch": tudo e desenhado direto em pixels
%   (insertShape/insertText), sem figuras do MATLAB, porque o sistema
%   grafico trava nesta maquina.
%
%   make_media('chunk093')
%   make_media('chunk093', 'fps', 30, 'slow', 2)   % camera lenta 2x

p = inputParser;
p.addParameter('fps', 30);
p.addParameter('slow', 1);
p.parse(varargin{:}); o = p.Results;
import org.opensim.modeling.*

here = fileparts(mfilename('fullpath'));
outDir = fullfile(here, '..', 'results');
S = load(fullfile(outDir, [tag '_traj.mat'])); tr = S.tr; cfg = S.cfg; M = S.M;
ref = healthyGaitReference();
n = numel(tr.t);

% ---------- cinematica direta: pontos do corpo no plano sagital ----------
model = Model(cfg.modelFile); s = model.initSystem();
CS = model.getCoordinateSet(); BS = model.getBodySet(); CG = model.getContactGeometrySet();
bodies = {'pelvis','transfemur','tibia_pylon','foot_r','femur_l','tibia_l','talus_l','torso'};
spheres = {'r_heel','r_toe1','l_heel','l_toe1'};
P = struct();
for k = [bodies spheres {'head'}], P.(k{1}) = zeros(n, 2); end
for i = 1:n
    for j = 1:numel(cfg.allCoords), CS.get(cfg.allCoords{j}).setValue(s, tr.q(i,j), false); end
    model.realizePosition(s);
    for k = 1:numel(bodies)
        v = BS.get(bodies{k}).getPositionInGround(s); P.(bodies{k})(i,:) = [v.get(0) v.get(1)];
    end
    v = BS.get('torso').findStationLocationInGround(s, Vec3(0, 0.55, 0)); P.head(i,:) = [v.get(0) v.get(1)];
    for k = 1:numel(spheres)
        g = CG.get(spheres{k});
        v = g.getFrame().findStationLocationInGround(s, g.get_location()); P.(spheres{k})(i,:) = [v.get(0) v.get(1)];
    end
end
writematrix([tr.t cell2mat(cellfun(@(k) P.(k), [bodies spheres {'head'}], 'UniformOutput', false))], ...
    fullfile(outDir, [tag '_pontos.csv']));

cR = [217 84 26]; cL = [0 114 189]; cT = [70 70 70]; cG = [150 150 150];

% =================== FIGURA-RESUMO ===================
W = 1800; H = 1400; img = uint8(255 * ones(H, W, 3));
img = txt(img, [W/2 18], sprintf(['%s  |  %.1f s em pe  |  v = %.2f m/s (saudavel: 1,2)  |  RMSE = %.1f°  |  ' ...
    'corr = %.2f  |  assimetria de apoio = %.0f%%'], tag, M.survivalTime, M.meanSpeed, M.rmseMean, M.corrMean, M.stanceSymmetry), ...
    24, [0 0 0], 'CenterTop');

% --- sequencia de bonecos ---
xs = [P.r_heel(:,1); P.l_toe1(:,1); P.head(:,1)];
pnl = panel([60 80 W-120 330], [min(xs)-0.1 max(xs)+0.1], [-0.05 1.75], true);
img = axesBox(img, pnl, 'Sequencia a cada 0,1 s  (laranja: protese passiva D, azul: perna intacta E)', 'x (m)', '', false);
img = shape(img, 'Line', [pnl.px(pnl.xl(1)) pnl.py(0) pnl.px(pnl.xl(2)) pnl.py(0)], [0 0 0], 2);
stepK = max(1, round(0.1 / mean(diff(tr.t))));
for i = 1:stepK:n
    a = 0.25 + 0.75 * i / n;
    img = drawStick(img, P, i, pnl, fade(cR, a), fade(cL, a), fade(cT, a), 2);
end

% --- angulos articulares x referencia saudavel ---
ph = mod(cfg.initPhase + tr.t / ref.cycleTime, 1);
idx = [7 10 11 12 14 15]; sig = {ref.hip, ref.knee, ref.ankle, ref.hip, ref.knee, ref.ankle};
off = [0.5 0.5 0.5 0 0 0]; rn = {'hipR','kneeR','ankR','hipL','kneeL','ankL'};
ttl = {'Quadril D (coto)','Joelho D (protese)','Tornozelo D (protese)','Quadril E','Joelho E','Tornozelo E'};
pw = (W - 120 - 2*70) / 3; ph0 = 470; phh = 230;
for j = 1:6
    col = mod(j-1, 3); row = floor((j-1) / 3);
    r = rad2deg(interp1((0:ref.N)'/ref.N, [sig{j}; sig{j}(1)], mod(ph + off(j), 1)));
    y = rad2deg(tr.q(:, idx(j)));
    yl = [min([r; y]) max([r; y])]; yl = yl + [-0.08 0.08] * diff(yl);
    pn = panel([60 + col*(pw+70), ph0 + row*(phh+80), pw, phh], [tr.t(1) tr.t(end)], yl, false);
    img = axesBox(img, pn, sprintf('%s   RMSE %.0f°', ttl{j}, M.rmse.(rn{j})), '', 'graus', row == 1);
    c = cR; if j > 3, c = cL; end
    img = polyline(img, pn, tr.t, r, [40 40 40], 2, true);
    img = polyline(img, pn, tr.t, y, c, 3, false);
    if j == 1
        img = txt(img, [pn.rect(1)+pn.rect(3)-10 pn.rect(2)+6], '- - saudavel (ref.)', 15, [0 0 0], 'RightTop');
        img = txt(img, [pn.rect(1)+pn.rect(3)-10 pn.rect(2)+26], '— simulado', 15, c, 'RightTop');
    end
end

% --- GRF, excitacoes, pelve ---
y0 = 1095; hh = 240;
g = tr.grf; pn = panel([60, y0, pw, hh], [tr.t(1) tr.t(end)], [0 max(g(:))*1.08 + 1], false);
img = axesBox(img, pn, 'Forca vertical do solo (N)', 't (s)', 'N', true);
img = polyline(img, pn, tr.t, g(:,1), cR, 2, false);
img = polyline(img, pn, tr.t, g(:,2), cL, 2, false);
img = txt(img, [pn.rect(1)+pn.rect(3)-10 pn.rect(2)+6], 'D (protese)', 15, cR, 'RightTop');
img = txt(img, [pn.rect(1)+pn.rect(3)-10 pn.rect(2)+26], 'E (intacta)', 15, cL, 'RightTop');

x1 = 60 + (pw+70) + 110; pn = panel([x1, y0, pw - 110, hh], [tr.t(1) tr.t(end)], [0.5 15.5], false);
E = tr.e(:,1:15)'; cm = round(255 * parula(256));
Ei = imresize(E, [hh round(pw - 110)], 'nearest'); Ei = min(256, max(1, round(Ei * 255) + 1));
x1 = round(x1); img(y0+1:y0+hh, x1+1:x1+round(pw-110), :) = reshape(uint8(cm(Ei, :)), [hh round(pw-110) 3]);
img = axesBox(img, pn, 'Excitacoes musculares (0 a 1)', 't (s)', '', true, false);
for k = 1:15
    img = txt(img, [x1 - 6, pn.py(16 - k) - 8], cfg.muscles{k}, 12, [0 0 0], 'RightTop');
end

x2 = 60 + 2*(pw+70); v = [NaN; diff(tr.q(:,4)) ./ diff(tr.t)]; v = movmean(v, 11, 'omitnan');
pn = panel([x2, y0, pw, hh], [tr.t(1) tr.t(end)], [0 1.6], false);
img = axesBox(img, pn, 'Pelve: altura (m) e velocidade v_x (m/s)', 't (s)', '', true);
img = shape(img, 'Line', [pn.px(tr.t(1)) pn.py(1.2) pn.px(tr.t(end)) pn.py(1.2)], cG, 1);
img = polyline(img, pn, tr.t, tr.q(:,5), cT, 3, false);
img = polyline(img, pn, tr.t, v, [119 172 48], 2, false);
img = txt(img, [pn.rect(1)+pn.rect(3)-10 pn.rect(2)+6], 'altura', 15, cT, 'RightTop');
img = txt(img, [pn.rect(1)+pn.rect(3)-10 pn.rect(2)+26], 'velocidade (cinza: 1,2 m/s)', 15, [119 172 48], 'RightTop');

figFile = fullfile(outDir, [tag '_resultado.png']); imwrite(img, figFile);
fprintf('figura: %s\n', figFile);

% =================== VIDEO ===================
VW = 1280; VH = 720; sc = 330;               % pixels por metro
vidFile = fullfile(outDir, [tag '_marcha.mp4']);
vw = VideoWriter(vidFile, 'MPEG-4'); vw.FrameRate = o.fps; vw.Quality = 90; open(vw);
tq = tr.t(1):1/(o.fps*o.slow):tr.t(end);
xc = movmean(P.pelvis(:,1), 25);               % camera suave
for tk = tq
    [~, i] = min(abs(tr.t - tk));
    pv = panel([0 0 VW VH], xc(i) + [-VW/2 VW/2]/sc, [-0.25, VH/sc - 0.25], true);
    fr = uint8(255 * ones(VH, VW, 3));
    gy = pv.py(0);
    fr(round(gy):end, :, :) = 235;
    for gx = floor(pv.xl(1)*4)/4 : 0.25 : pv.xl(2)
        fr = shape(fr, 'Line', [pv.px(gx) gy pv.px(gx) gy+14], cG, 2);
    end
    fr = shape(fr, 'Line', [0 gy VW gy], [0 0 0], 3);
    fr = drawStick(fr, P, i, pv, cR, cL, cT, 7);
    sc_g = 0.35 / (9.81 * 70);                  % 1 peso corporal = 0,35 m de seta
    feet = {[P.r_heel(i,1) P.r_toe1(i,1)], [P.l_heel(i,1) P.l_toe1(i,1)]}; cc = {cR, cL};
    for k = 1:2
        if tr.grf(i,k) > 20
            x = mean(feet{k}); L = tr.grf(i,k) * sc_g;
            fr = shape(fr, 'Line', [pv.px(x) gy pv.px(x) pv.py(L)], cc{k}, 4);
            fr = shape(fr, 'FilledPolygon', [pv.px(x)-9 pv.py(L)+2 pv.px(x)+9 pv.py(L)+2 pv.px(x) pv.py(L)-14], cc{k}, 1);
        end
    end
    fr = txt(fr, [20 15], sprintf('t = %.2f s     v media = %.2f m/s     %s', tr.t(i), M.meanSpeed, tag), 26, [0 0 0], 'LeftTop');
    fr = txt(fr, [20 55], 'laranja: protese passiva (joelho mola+amortecedor, pe ESAR)   azul: perna intacta   setas: forca do solo', 18, cT, 'LeftTop');
    if o.slow ~= 1, fr = txt(fr, [VW-20 15], sprintf('camera lenta %gx', o.slow), 22, cT, 'RightTop'); end
    writeVideo(vw, fr);
end
close(vw);
fprintf('video: %s (%d quadros)\n', vidFile, numel(tq));
end

% =================== auxiliares de desenho ===================
function pn = panel(rect, xl, yl, equalAxes)
% rect = [x y w h] em pixels; xl/yl = faixa de dados
if equalAxes      % mesma escala em x e y (para os bonecos)
    sc = min(rect(3) / diff(xl), rect(4) / diff(yl));
    xl = mean(xl) + [-0.5 0.5] * rect(3) / sc;
    yl = yl(1) + [0 1] * rect(4) / sc;
end
pn.rect = rect; pn.xl = xl; pn.yl = yl;
pn.px = @(x) rect(1) + (x - xl(1)) / diff(xl) * rect(3);
pn.py = @(y) rect(2) + rect(4) - (y - yl(1)) / diff(yl) * rect(4);
end

function img = shape(img, kind, pos, c, w)
if any(~isfinite(pos(:))), return; end
if strcmp(kind, 'Line')
    img = insertShape(img, 'Line', pos, 'Color', c, 'LineWidth', w, 'SmoothEdges', true);
elseif strcmp(kind, 'FilledPolygon') || strcmp(kind, 'FilledCircle')
    img = insertShape(img, kind, pos, 'Color', c, 'Opacity', w, 'SmoothEdges', true);
else
    img = insertShape(img, kind, pos, 'Color', c, 'LineWidth', w, 'SmoothEdges', true);
end
end

function img = txt(img, pos, str, fs, c, anchor)
img = insertText(img, pos, str, 'FontSize', fs, 'TextColor', c, 'BoxOpacity', 0, 'AnchorPoint', anchor);
end

function c = fade(c, a)
c = round(c * a + 255 * (1 - a));
end

function img = polyline(img, pn, x, y, c, w, dashed)
ok = isfinite(x) & isfinite(y); x = x(ok); y = y(ok);
X = pn.px(x(:)); Y = pn.py(y(:));
if dashed
    seg = 4; % pontos por traco
    for k = 1:round(1.6*seg):numel(X)-1
        e = min(numel(X), k + seg);
        pts = [X(k:e) Y(k:e)]'; img = shape(img, 'Line', pts(:)', c, w);
    end
else
    pts = [X Y]'; img = shape(img, 'Line', pts(:)', c, w);
end
end

function img = axesBox(img, pn, ttl, xlab, ylab, showX, showY)
if nargin < 7, showY = true; end
r = pn.rect;
img = insertShape(img, 'Rectangle', r, 'Color', [120 120 120], 'LineWidth', 1);
img = txt(img, [r(1) r(2) - 30], ttl, 19, [0 0 0], 'LeftTop');
yt = niceTicks(pn.yl); if ~showY, yt = []; end
for v = yt
    y = pn.py(v);
    img = shape(img, 'Line', [r(1) y r(1)+r(3) y], [225 225 225], 1);
    img = txt(img, [r(1) - 6, y - 9], num2str(v), 13, [60 60 60], 'RightTop');
end
xt = niceTicks(pn.xl);
for v = xt
    x = pn.px(v);
    img = shape(img, 'Line', [x r(2)+r(4) x r(2)+r(4)+5], [120 120 120], 1);
    if showX || isempty(ylab), img = txt(img, [x, r(2)+r(4)+7], num2str(v), 13, [60 60 60], 'CenterTop'); end
end
if ~isempty(xlab) && showX, img = txt(img, [r(1)+r(3), r(2)+r(4)+26], xlab, 14, [60 60 60], 'RightTop'); end
if ~isempty(ylab), img = txt(img, [r(1) - 50, r(2) - 4], ylab, 13, [60 60 60], 'LeftTop'); end
end

function t = niceTicks(lim)
d = diff(lim) / 5; m = 10^floor(log10(d)); d = m * [1 2 5 10]; d = d(find(d >= diff(lim)/6, 1));
t = ceil(lim(1)/d)*d : d : lim(2);
t = round(t, 6);
end

function img = drawStick(img, P, i, pn, cR, cL, cT, lw)
pt = @(nm) [pn.px(P.(nm)(i,1)) pn.py(P.(nm)(i,2))];
line = @(img, names, c, w) shape(img, 'Line', reshape(cell2mat(cellfun(pt, names, 'UniformOutput', false)')', 1, []), c, w);
poly = @(names) reshape(cell2mat(cellfun(pt, names, 'UniformOutput', false)')', 1, []);
% perna intacta (atras), tronco, protese (frente, lado direito visto de lado)
img = line(img, {'pelvis','femur_l','tibia_l','talus_l'}, cL, lw);
img = shape(img, 'FilledPolygon', poly({'talus_l','l_heel','l_toe1'}), cL, 0.6);
img = line(img, {'pelvis','torso','head'}, cT, lw + 2);
img = line(img, {'pelvis','transfemur','tibia_pylon','foot_r'}, cR, lw);
img = shape(img, 'FilledPolygon', poly({'foot_r','r_heel','r_toe1'}), cR, 0.6);
if lw >= 5
    h = pt('head'); rh = 0.11 * (pn.rect(3) / diff(pn.xl));
    img = shape(img, 'FilledCircle', [h(1) h(2) - rh*0.8 rh], [255 255 255], 1);
    img = shape(img, 'Circle', [h(1) h(2) - rh*0.8 rh], cT, 3);
    for nm = {'femur_l','tibia_l','talus_l','transfemur','tibia_pylon','foot_r','pelvis'}
        q = pt(nm{1}); img = shape(img, 'FilledCircle', [q lw*0.9], [255 255 255], 1);
        img = shape(img, 'Circle', [q lw*0.9], cT, 2);
    end
end
end
