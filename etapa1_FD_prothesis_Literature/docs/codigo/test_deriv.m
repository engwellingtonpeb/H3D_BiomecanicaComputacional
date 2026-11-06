% Compara integradores do MATLAB usando o OpenSim apenas para ydot = f(t,y)
import org.opensim.modeling.*
cfg = gaitConfig('randomInitPhase', false);
env = ProstheticGaitEnv(cfg); reset(env);
m = env.model; s = org.opensim.modeling.State(env.state);
y0 = s.getY().getAsMat();
fprintf('nY=%d nQ=%d nU=%d nZ=%d\n', numel(y0), s.getNQ(), s.getNU(), s.getNZ());
f = @(t,y) deriv(m, s, t, y);
tic; for k=1:200, d = f(0, y0); end; fprintf('deriv: %.2f ms\n', toc/200*1000);
global NF
for solver = {@ode45, @ode23, @ode15s, @ode23t, @ode23tb}
  NF = 0; tic;
  opts = odeset('RelTol',1e-3,'AbsTol',1e-4);
  try
    [T,Y] = solver{1}(f, [0 0.1], y0, opts);
    fprintf('%s: 0.1 s em %.2f s, %d avaliacoes, %d passos, ty fim=%.3f\n', func2str(solver{1}), toc, NF, numel(T), Y(end,5));
  catch e, fprintf('%s falhou: %s\n', func2str(solver{1}), e.message); end
end
function d = deriv(m, s, t, y)
global NF; NF = NF + 1;
s.setTime(t); s.setY(org.opensim.modeling.Vector.createFromMat(y));
m.realizeAcceleration(s);
d = s.getYDot().getAsMat();
end
