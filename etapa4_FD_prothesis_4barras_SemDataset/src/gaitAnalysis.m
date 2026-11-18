function A = gaitAnalysis(trajFile, kind)
%GAITANALYSIS CIR x fase x GRF e assimetria de uma trajetoria simulada.
%   A = gaitAnalysis(trajFile, kind), kind = 'fourbar' (etapa 3) ou 'pin'
%   (protese monocentrica, etapas 1-2).
%
%   Reproduz a trajetoria no modelo OpenSim correspondente (cinematica
%   direta) para obter, a cada 10 ms:
%     - CIR da perna protetica em relacao a coxa: referencial da coxa
%       (cirT, x anterior / y superior, m) e no solo (cirG)
%       * 4 barras: intersecao dos dois elos (tabela fourBarMap)
%       * pino: centro fixo da articulacao
%     - forca de reacao do solo (GRF) e centro de pressao (COP) de cada pe
%     - momento da GRF protetica em torno do CIR (Mcir, N.m) e o braco de
%       alavanca com sinal (dcir, m): > 0 = linha de forca ANTERIOR ao CIR
%       = momento extensor (joelho estavel); < 0 = flexor (tende a dobrar)
%   e, por passada (contato inicial: GRF vertical cruza 20 N), a fase do
%   ciclo de cada lado e os parametros espaco-temporais/cineticos usados no
%   indice de simetria SI = 100 (P - I) / (0,5 (P + I)), P = protese (D),
%   I = perna intacta (E).
import org.opensim.modeling.*
S = load(trajFile); tr = S.tr; cfg = S.cfg;
A.file = trajFile; A.kind = kind; A.t = tr.t;
m = Model(cfg.modelFile); m.setUseVisualizer(false); s = m.initSystem();
cs = m.getCoordinateSet(); bs = m.getBodySet();
BW = m.getTotalMass(s) * 9.81; A.BW = BW;
if strcmp(kind, 'fourbar')
    q = tr.qraw; names = [cfg.allCoords cfg.fourBar.depCoords]; q = [q tr.dep];
    F = fourBarMap(cfg); ok = all(isfinite(F.cir), 2);
    fCir = griddedInterpolant(flipud(F.link(ok)), flipud(F.cir(ok,:)), 'pchip');
else
    q = tr.q; names = cfg.allCoords;
    F = fourBarMap(gaitConfig4S());                 % so para o centro do pino (mesmo referencial)
end
A.pinCenter = F.pinCenter;
n = numel(tr.t); dt = 0.01;
qd = [zeros(1, size(q,2)); diff(q)] / dt;            % velocidades (dif. finitas) so p/ o contato
C = cellfun(@(x) cs.get(x), names, 'UniformOutput', false);
tf = bs.get('transfemur');
hc = {HuntCrossleyForce.safeDownCast(m.getForceSet().get('foot_r')), HuntCrossleyForce.safeDownCast(m.getForceSet().get('foot_l'))};
heel = {bs.get('foot_r'), bs.get('calcn_l')};
A.cirT = zeros(n,2); A.cirG = zeros(n,2); A.cop = nan(n,2); A.heelX = zeros(n,2);
for k = 1:n
    for j = 1:numel(C), C{j}.setValue(s, q(k,j), false); C{j}.setSpeedValue(s, qd(k,j)); end
    m.realizeDynamics(s);
    if strcmp(kind, 'fourbar'), cT = fCir(tr.link(k)); else, cT = F.pinCenter; end
    p = tf.findStationLocationInGround(s, Vec3(cT(1), cT(2), 0));
    A.cirT(k,:) = cT; A.cirG(k,:) = [p.get(0) p.get(1)];
    for i = 1:2
        v = hc{i}.getRecordValues(s);
        if v.get(1) < -20, A.cop(k,i) = v.get(5) / v.get(1); end      % COP x = Tz/Fy (forca sobre o solo)
        h = heel{i}.getPositionInGround(s); A.heelX(k,i) = h.get(0);
    end
end
% forcas: as registradas na simulacao (exatas); COP: da reconstrucao
A.grf = [tr.grf3(:,1:2) tr.grf3(:,4:5)];               % [FxR FyR FxL FyL] (N), sobre o pe
if isfield(tr, 'copR') && ~isempty(tr.copR), A.cop(:,1) = tr.copR(:,1); A.cop(:,2) = tr.copL(:,1); end
Fx = A.grf(:,1); Fy = A.grf(:,2);
rx = A.cop(:,1) - A.cirG(:,1); ry = 0 - A.cirG(:,2);
A.Mcir = rx .* Fy - ry .* Fx;                          % momento da GRF protetica em torno do CIR (N.m)
A.dcir = A.Mcir ./ max(hypot(Fx, Fy), 1);              % braco com sinal (m)
A.Mcir(Fy < 20) = NaN; A.dcir(Fy < 20) = NaN;
A.kneeFlex = rad2deg(tr.q(:,[10 14]));                 % [protese intacta], graus (negativo = flexao)
A.hip = rad2deg(tr.q(:,[7 12]));

% ---------- passadas e fase ----------
thr = 20; side = 'RL';
for i = 1:2
    fz = A.grf(:, 2*i);
    hs = find(fz(2:end) >= thr & fz(1:end-1) < thr) + 1;
    hs = hs(A.t(hs) >= 1.0);                            % descarta o transitorio inicial
    A.hs.(side(i)) = hs;
    ph = nan(n,1);
    for j = 1:numel(hs)-1
        T = A.t(hs(j+1)) - A.t(hs(j)); if T < 0.5 || T > 2.0, continue; end
        ph(hs(j):hs(j+1)-1) = (A.t(hs(j):hs(j+1)-1) - A.t(hs(j))) / T;
    end
    A.phase.(side(i)) = ph;
end

% ---------- curvas normalizadas no ciclo da protese (101 pontos) ----------
pct = (0:100)'/100; hs = A.hs.R;
vars = struct('kneeFlex', A.kneeFlex(:,1), 'cirX', 1000*A.cirT(:,1), 'cirY', 1000*A.cirT(:,2), ...
    'grfV', Fy/BW, 'grfAP', Fx/BW, 'dcir', 1000*A.dcir, 'Mcir', A.Mcir, ...
    'grfV_L', A.grf(:,4)/BW, 'kneeFlex_L', A.kneeFlex(:,2));
fn = fieldnames(vars);
for f = fn', A.cyc.(f{1}) = []; end
A.stanceEnd = [];
for j = 1:numel(hs)-1
    i0 = hs(j); i1 = hs(j+1); T = A.t(i1) - A.t(i0); if T < 0.5 || T > 2.0, continue; end
    tt = (A.t(i0:i1) - A.t(i0)) / T;
    for f = fn'
        % NaN fica NaN (ex.: braco da GRF no balanco, sem contato)
        A.cyc.(f{1})(:,end+1) = interp1(tt, vars.(f{1})(i0:i1), pct, 'linear');
    end
    off = find(Fy(i0:i1) < thr, 1); A.stanceEnd(end+1) = (off - 1) / (i1 - i0);
end
A.pct = 100 * pct;

% ---------- parametros por lado e simetria ----------
P = struct();
for i = 1:2
    sd = side(i); o = 3 - i; h = A.hs.(sd); ho = A.hs.(side(o)); fz = A.grf(:, 2*i); fx = A.grf(:, 2*i-1);
    v = struct('stride',[],'stance',[],'swing',[],'stancePct',[],'stepTime',[],'stepLen',[], ...
               'peakV',[],'impV',[],'impBrake',[],'impProp',[],'kneeROM',[],'hipROM',[]);
    for j = 1:numel(h)-1
        i0 = h(j); i1 = h(j+1); T = A.t(i1) - A.t(i0); if T < 0.5 || T > 2.0, continue; end
        off = i0 + find(fz(i0:i1) < thr, 1) - 1;
        v.stride(end+1) = T; v.stance(end+1) = A.t(off) - A.t(i0); v.swing(end+1) = A.t(i1) - A.t(off);
        v.stancePct(end+1) = 100 * v.stance(end) / T;
        prev = ho(find(ho < i0 & ho > i0 - round(T/dt), 1, 'last'));   % contato do outro pe dentro desta passada
        if ~isempty(prev), v.stepTime(end+1) = A.t(i0) - A.t(prev); end
        v.stepLen(end+1) = A.heelX(i0, i) - A.heelX(i0, o);   % distancia entre os pes no contato
        st = i0:off; v.peakV(end+1) = max(fz(st)) / BW;
        v.impV(end+1) = sum(fz(st)) * dt / BW;
        v.impBrake(end+1) = sum(min(fx(st), 0)) * dt / BW; v.impProp(end+1) = sum(max(fx(st), 0)) * dt / BW;
        v.kneeROM(end+1) = range(A.kneeFlex(i0:i1, i)); v.hipROM(end+1) = range(A.hip(i0:i1, i));
    end
    P.(sd) = v;
end
names = fieldnames(P.R); A.params = table();
for k = 1:numel(names)
    a = mean(P.R.(names{k}), 'omitnan'); b = mean(P.L.(names{k}), 'omitnan');
    A.params(k, :) = {string(names{k}), a, b, 100 * (a - b) / (0.5 * (a + b))};
end
A.params.Properties.VariableNames = {'param', 'protese_D', 'intacta_E', 'SI_pct'};
A.nStrides = [numel(P.R.stride) numel(P.L.stride)];
end
