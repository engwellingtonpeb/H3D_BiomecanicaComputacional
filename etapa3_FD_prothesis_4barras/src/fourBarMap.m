function F = fourBarMap(cfg, rebuild)
%FOURBARMAP Cinematica do joelho policentrico de 4 barras (mecanismo de 1 GDL).
%   Varre o angulo do elo 4x1 (knee_angle_r) e tabela, para cada valor:
%     dep    coordenadas dependentes [dist_4_1_rz 4x2_dist_rz] (restricao fechada)
%     flex   flexao real coxa-perna (rad; negativo = flexao, como knee_angle_l)
%     cir    centro instantaneo de rotacao (CIR) da perna em relacao a coxa,
%            no referencial da coxa (transfemur): [x y] em m, x anterior, y superior
%     piv    pivos [A B C D] (x,y no referencial da coxa): elo 4_1 = A-B, elo 4x2 = D-C
%   Como o mecanismo tem 1 GDL, tudo isso e funcao so do angulo do elo:
%   durante a simulacao basta interpolar a tabela.
%   Tambem guarda o centro do pino do joelho da protese monocentrica
%   (etapas 1-2) no mesmo referencial, para comparacao.
if nargin < 2, rebuild = false; end
if ~rebuild && isfile(cfg.fourBar.mapFile), F = load(cfg.fourBar.mapFile); return, end
import org.opensim.modeling.*
m = Model(cfg.modelFile); m.setUseVisualizer(false); s = m.initSystem();
cs = m.getCoordinateSet(); bs = m.getBodySet(); js = m.getJointSet();
lk = cs.get(cfg.fourBar.linkCoord); d1 = cs.get(cfg.fourBar.depCoords{1}); d2 = cs.get(cfg.fourBar.depCoords{2});
tf = bs.get('transfemur'); tb = bs.get('tibia_pylon');
con = PointConstraint.safeDownCast(m.getConstraintSet().get('close_fourbar'));
pD = con.get_location_body_2();                      % ponto no corpo prox
F.link = (0:-0.005:-3.3)';     % rad (0 = extensao total; ate ~125 graus de flexao; na marcha < 75)
n = numel(F.link); F.dep = zeros(n,2); F.flex = zeros(n,1); F.cir = nan(n,2); F.piv = zeros(n,8);
for i = 1:n
    lk.setValue(s, F.link(i), false); m.assemble(s); m.realizePosition(s);
    F.dep(i,:) = [d1.getValue(s) d2.getValue(s)];
    Rt = rotOf(tf, s); Rb = rotOf(tb, s); Rr = Rt' * Rb;
    F.flex(i) = atan2(Rr(2,1), Rr(1,1));
    toThigh = @(p) (Rt' * (vec(p) - vec(tf.getPositionInGround(s))))';
    A = toThigh(js.get('4x1_prox').getParentFrame().getPositionInGround(s));
    B = toThigh(js.get('dist_4_1').getParentFrame().getPositionInGround(s));
    C = toThigh(js.get('4x2_dist').getParentFrame().getPositionInGround(s));
    D = toThigh(bs.get('prox').findStationLocationInGround(s, pD));
    F.piv(i,:) = [A(1:2) B(1:2) C(1:2) D(1:2)];
    F.cir(i,:) = lineX(A(1:2), B(1:2), D(1:2), C(1:2));
end
% descarta amostras em que a montagem nao convergiu (flexao deve cair sempre)
ok = [true; diff(F.flex) < 0] & [true; diff(F.link) < 0];
for fn = {'link','dep','flex','cir','piv'}, F.(fn{1}) = F.(fn{1})(cummin(ok) > 0, :); end
% referencia: joelho de pino da protese monocentrica (etapas 1-2)
m0 = Model(fullfile(fileparts(cfg.modelFile), 'OS4_gait14dof15musc_2Act_RTFP_DynAct.osim'));
m0.setUseVisualizer(false); s0 = m0.initSystem(); m0.realizePosition(s0);
tf0 = m0.getBodySet().get('transfemur'); R0 = rotOf(tf0, s0);
k0 = vec(m0.getJointSet().get('knee_r').getParentFrame().getPositionInGround(s0)) - vec(tf0.getPositionInGround(s0));
k0 = (R0' * k0)'; F.pinCenter = k0(1:2);
% tornozelo protetico (para a linha de carga), no referencial da coxa em extensao
lk.setValue(s, 0, false); m.assemble(s); m.realizePosition(s);
F.note = 'link = knee_angle_r (elo 4x1); flex = coxa-perna; cir/piv/pinCenter no referencial da coxa (x anterior, y superior), m';
save(cfg.fourBar.mapFile, '-struct', 'F');
end

function R = rotOf(b, s)
T = b.getTransformInGround(s).R().asMat33(); R = zeros(3);
for i = 0:2, for j = 0:2, R(i+1,j+1) = T.get(i,j); end, end
end
function v = vec(p), v = [p.get(0); p.get(1); p.get(2)]; end
function X = lineX(p1, p2, p3, p4)
% intersecao das retas p1-p2 e p3-p4 (2D); NaN se quase paralelas
d1 = p2 - p1; d2 = p4 - p3; den = d1(1)*d2(2) - d1(2)*d2(1);
if abs(den) < 1e-9 * norm(d1) * norm(d2), X = [NaN NaN]; return, end
t = ((p3(1)-p1(1))*d2(2) - (p3(2)-p1(2))*d2(1)) / den;
X = p1 + t * d1;
end
