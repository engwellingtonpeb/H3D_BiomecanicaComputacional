function sweep(varargin)
% sweep('param',{v1,v2,...}, extras...) roda o controlador-base (acao zero, sem assistencia)
base = {'randomInitPhase', false, 'assistStart', 0};
name = varargin{1}; vals = varargin{2}; extra = varargin(3:end);
if ~iscell(vals), vals = num2cell(vals); end
for iv = 1:numel(vals)
  v = vals{iv};
  for ph = [0 0.5]
    cfg = gaitConfig(base{:}, 'initPhase', ph, extra{:}, name, v);
    env = ProstheticGaitEnv(cfg); env.record = true; reset(env);
    P = 0;
    for k = 1:cfg.maxSteps
        [~, ~, done, info] = step(env, zeros(15,1)); P = P + info.pose;
        if done, break; end
    end
    tr = env.traj; qe = rad2deg(tr.q(end,:));
    fprintf('%s=%s ph=%.1f: %3d passos | pose=%.2f | vx=%.2f | tilt=%.2f ty=%.2f | fim: hipR %.0f kneeR %.0f ankR %.0f hipL %.0f kneeL %.0f ankL %.0f\n', ...
        name, mat2str(v), ph, k, P/k, tr.q(end,4)/tr.t(end), tr.q(end,1), tr.q(end,5), qe([7 10 11 12 14 15]));
  end
end
end
