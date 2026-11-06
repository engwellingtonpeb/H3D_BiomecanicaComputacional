% Testa o ambiente com acao zero (so controlador-base) com e sem assistencia
for assist = [1 0.5 0]
    cfg = gaitConfig('randomInitPhase', false, 'initPhase', 0.0, 'assistStart', assist);
    env = ProstheticGaitEnv(cfg);
    env.record = true;
    obs = reset(env);
    R = 0; tic;
    for k = 1:cfg.maxSteps
        [obs, r, done, info] = step(env, zeros(15,1));
        R = R + r;
        if done, break; end
    end
    el = toc;
    tr = env.traj;
    vx = (tr.q(end,4) - 0) / tr.t(end);
    fprintf('assist=%.1f: %d passos (%.2f s sim) em %.1f s | R=%.1f | vx med=%.2f | ty fim=%.2f | tilt fim=%.2f\n', ...
        assist, k, tr.t(end), el, R, vx, tr.q(end,5), tr.q(end,1));
    fprintf('   pose=%.2f speed=%.2f upright=%.2f effort=%.3f | GRF max r/l = %.0f/%.0f N\n', ...
        info.pose, info.speed, info.upright, info.effort, max(tr.grf(:,1)), max(tr.grf(:,2)));
    fprintf('   faixa (graus): hipR [%.0f %.0f] kneeR [%.0f %.0f] ankR [%.0f %.0f] hipL [%.0f %.0f] kneeL [%.0f %.0f] ankL [%.0f %.0f]\n', ...
        rad2deg([min(tr.q(:,7)) max(tr.q(:,7)) min(tr.q(:,10)) max(tr.q(:,10)) min(tr.q(:,11)) max(tr.q(:,11)) ...
                 min(tr.q(:,12)) max(tr.q(:,12)) min(tr.q(:,14)) max(tr.q(:,14)) min(tr.q(:,15)) max(tr.q(:,15))]));
end
