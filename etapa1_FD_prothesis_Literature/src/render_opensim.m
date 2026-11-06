function render_opensim(tag, varargin)
%RENDER_OPENSIM Video da marcha no visualizador 3D do OpenSim (Simbody
%   visualizer, com os ossos, a protese e os musculos do modelo). Reproduz a
%   trajetoria salva em results/<tag>_traj.mat, captura cada quadro da
%   janela do visualizador e grava results/<tag>_opensim.mp4.
%   A janela do visualizador precisa ficar visivel (nao minimizada, sem
%   outras janelas por cima) durante a gravacao.
%
%   render_opensim('cmp_chunk129_lr1e-4_ckpt_p00')
%   render_opensim(tag, 'fps', 30, 'maxTime', 10, 'testFrame', true)

p = inputParser;
p.addParameter('fps', 30);
p.addParameter('maxTime', inf);
p.addParameter('testFrame', false);      % so 1 quadro (para conferir enquadramento)
p.addParameter('camDist', 2.6);          % distancia da camera (m)
p.parse(varargin{:}); o = p.Results;
import org.opensim.modeling.*

here = fileparts(mfilename('fullpath'));
outDir = fullfile(here, '..', 'results');
S = load(fullfile(outDir, [tag '_traj.mat'])); tr = S.tr; cfg = S.cfg; M = S.M;

ModelVisualizer.addDirToGeometrySearchPaths(fullfile(fileparts(cfg.modelFile), 'Geometry'));
ModelVisualizer.addDirToGeometrySearchPaths('C:\OpenSim 4.4\Geometry');   % ossos padrao (.vtp)
model = Model(cfg.modelFile);
model.setUseVisualizer(true);
s = model.initSystem();
sviz = model.updVisualizer().updSimbodyVisualizer();
sviz.setShowSimTime(true);
sviz.setShowFrameRate(false);
CS = model.getCoordinateSet();
capt = fullfile(here, 'capture_window.ps1');
frDir = fullfile(tempdir, ['osimframes_' tag]); if ~exist(frDir, 'dir'), mkdir(frDir); end

tEnd = min(tr.t(end), o.maxTime);
tq = tr.t(1):1/o.fps:tEnd;
if o.testFrame, tq = tr.t(round(end/2)); end
xc = movmean(tr.q(:,4), 25);               % camera acompanha a pelve (suave)
for k = 1:numel(tq)
    [~, i] = min(abs(tr.t - tq(k)));
    s.setTime(tr.t(i));
    for j = 1:numel(cfg.allCoords), CS.get(cfg.allCoords{j}).setValue(s, tr.q(i,j), false); end
    model.realizePosition(s);
    % vista lateral (plano sagital), olhando o lado direito (protese)
    sviz.setCameraTransform(Transform(Vec3(xc(i), 0.8, o.camDist)));
    model.getVisualizer().show(s);
    if k == 1, pause(1.5); else, pause(0.03); end
    f = fullfile(frDir, sprintf('f%05d.png', k));
    front = ''; if k == 1, front = ' -front'; end
    for tent = 1:4   % confere se o quadro mostra a cena (ceu azul no alto); se outra janela cobriu, captura de novo
        [st, msg] = system(sprintf('powershell -NoProfile -ExecutionPolicy Bypass -File "%s" -out "%s"%s', capt, f, front));
        if st ~= 0, error('captura falhou: %s', msg); end
        I = double(imread(f)); sky = I(60:120, round(end*0.7):end-10, :);
        if mean(sky(:,:,3), 'all') > mean(sky(:,:,1), 'all') + 40, break; end
        fprintf('quadro %d coberto por outra janela, capturando de novo\n', k);
        front = ' -front'; model.getVisualizer().show(s); pause(0.3);
    end
    if mod(k, 60) == 0, fprintf('quadro %d/%d\n', k, numel(tq)); end
end

if o.testFrame
    copyfile(f, fullfile(outDir, [tag '_opensim_teste.png']));
    fprintf('teste: %s\n', fullfile(outDir, [tag '_opensim_teste.png']));
    return
end
vidFile = fullfile(outDir, [tag '_opensim.mp4']);
vw = VideoWriter(vidFile, 'MPEG-4'); vw.FrameRate = o.fps; vw.Quality = 90; open(vw);
I0 = imread(fullfile(frDir, 'f00001.png')); sz = size(I0); sz = 2*floor(sz(1:2)/2);
for k = 1:numel(tq)
    I = imread(fullfile(frDir, sprintf('f%05d.png', k)));
    I = I(1:sz(1), 1:sz(2), :);
    I = insertText(I, [10 10], sprintf('v media = %.2f m/s  |  protese passiva: perna direita  |  %s', M.meanSpeed, tag), ...
        'FontSize', 16, 'BoxOpacity', 0.4);
    writeVideo(vw, I);
end
close(vw);
fprintf('video: %s (%d quadros)\n', vidFile, numel(tq));
end
