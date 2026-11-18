classdef FreeGaitEnv < rl.env.MATLABEnvironment
    %FREEGAITENV ETAPA 4. Marcha com o joelho protetico POLICENTRICO (4 barras,
    %   50_Direita_Final.osim) SEM DATASET. Mesma dinamica direta das etapas
    %   anteriores (OpenSim avalia ydot, ode23 do MATLAB integra, ZOH 10 ms) e
    %   mesmo tratamento do mecanismo (tabela fourBarMap, reprojecao das
    %   coordenadas dependentes a cada passo).
    %
    %   Acao (15) -> excitacao direta de cada musculo: e = (u+1)/2 (filtrada).
    %   Observacao: mesmas grandezas da etapa 3, mas as 8 entradas da
    %   referencia do dataset ficam em zero (e as 2 de fase tambem, a menos que
    %   cfg.clock = true: relogio interno de periodo livre).
    %   Recompensa: so plausibilidade fisiologica (ver gaitConfig4S).
    %   Protese PASSIVA: joelho e tornozelo apenas com mola + amortecedor.

    properties
        cfg
        episodeCount = 0
        record = false
        traj = struct()
        fixedInit = 0       % > 0: usa sempre esse estado do banco (avaliacao)
    end

    properties (Transient)
        model; state; ctrlFns; coords; muscles; contactForces
        actIdx; depC; heelBody; pool
        fb
        built = false
    end

    properties (Access = private)
        t = 0; k = 0
        u_prev; e_prev
        bodyWeight = 1
        wF                  % pesos do custo metabolico (Fmax normalizado)
        contact = [false false]; lastStrike = 0; lastStrikeT = [NaN NaN]; stepT = [NaN NaN]
        nSteps = 0
    end

    methods
        function this = FreeGaitEnv(cfg)
            if nargin < 1, cfg = gaitConfig4S(); end
            nObs = FreeGaitEnv.obsSize(cfg);
            oi = rlNumericSpec([nObs 1]); oi.Name = 'gait_obs';
            ai = rlNumericSpec([numel(cfg.muscles) 1], 'LowerLimit', -1, 'UpperLimit', 1);
            ai.Name = 'muscle_excitations';
            this = this@rl.env.MATLABEnvironment(oi, ai);
            this.cfg = cfg;
        end

        function build(this)
            import org.opensim.modeling.*
            c = this.cfg;
            m = Model(c.modelFile); m.setUseVisualizer(false);
            cs = m.getCoordinateSet();
            for n = c.lockedCoords
                cs.get(n{1}).setDefaultLocked(true); cs.get(n{1}).setDefaultValue(0);
            end
            for n = {'knee_actuator','ankle_actuator'}
                m.updForceSet().get(n{1}).set_appliesForce(false);
            end
            P = c.prosthesis;
            for n = fieldnames(P)'
                pp = P.(n{1});
                sp = SpringGeneralizedForce(pp.coord); sp.setName(['passive_' n{1}]);
                sp.setStiffness(pp.stiffness); sp.setRestLength(pp.rest); sp.setViscosity(pp.damping);
                m.addForce(sp);
            end
            brain = PrescribedController();
            acts = m.getActuators(); nA = acts.getSize(); names = cell(nA,1);
            for i = 0:nA-1
                a = acts.get(i); names{i+1} = char(a.getName());
                brain.addActuator(a); brain.prescribeControlForActuator(names{i+1}, Constant(0));
            end
            m.addController(brain);
            s = m.initSystem();
            fs = brain.get_ControlFunctions();
            this.ctrlFns = cell(nA,1);
            for i = 0:nA-1, this.ctrlFns{i+1} = Constant.safeDownCast(fs.get(i)); end
            this.actIdx = cellfun(@(x) find(strcmp(names, x)), c.muscles);
            this.coords = struct();
            for n = c.allCoords, this.coords.(n{1}) = cs.get(n{1}); end
            ms = m.getMuscles();
            this.muscles = cell(numel(c.muscles),1);
            for i = 1:numel(c.muscles), this.muscles{i} = ms.get(c.muscles{i}); end
            this.contactForces = {HuntCrossleyForce.safeDownCast(m.getForceSet().get('foot_r')), ...
                                  HuntCrossleyForce.safeDownCast(m.getForceSet().get('foot_l'))};
            this.heelBody = {m.getBodySet().get('foot_r'), m.getBodySet().get('calcn_l')};
            this.bodyWeight = m.getTotalMass(s) * 9.80665;
            this.wF = c.Fmax(:) / sum(c.Fmax);
            F = fourBarMap(c); this.fb = F;
            this.fb.fFlex = griddedInterpolant(flipud(F.link), flipud(F.flex), 'pchip');
            this.fb.fDflex = griddedInterpolant(flipud(F.link), flipud(gradient(F.flex, F.link)), 'pchip');
            for j = 1:2
                this.fb.fDep{j} = griddedInterpolant(flipud(F.link), flipud(F.dep(:,j)), 'pchip');
                this.fb.fDdep{j} = griddedInterpolant(flipud(F.link), flipud(gradient(F.dep(:,j), F.link)), 'pchip');
            end
            ok = all(isfinite(F.cir), 2);
            this.fb.fCir = griddedInterpolant(flipud(F.link(ok)), flipud(F.cir(ok,:)), 'pchip');
            this.depC = {cs.get(c.fourBar.depCoords{1}), cs.get(c.fourBar.depCoords{2})};
            this.pool = load(c.initStatesFile);
            this.model = m; this.state = s; this.built = true;
        end

        function obs = reset(this)
            if ~this.built, this.build(); end
            c = this.cfg; m = this.model;
            this.episodeCount = this.episodeCount + 1;
            for attempt = 1:6
                s = m.initializeState();
                if this.fixedInit > 0, i = this.fixedInit; else, i = randi(size(this.pool.q, 1)); end
                if this.initPose(s, i, attempt < 6 && this.fixedInit == 0), break; end
            end
            this.state = s; this.t = 0; this.k = 0;
            this.u_prev = zeros(numel(c.muscles),1);
            this.e_prev = this.pool.e(i,:)';
            g = this.grf3(s);
            this.contact = [g(2) g(5)] >= c.grfThr;
            this.lastStrike = 0; this.lastStrikeT = [NaN NaN]; this.stepT = [NaN NaN]; this.nSteps = 0;
            if this.record, this.traj = struct('t',[],'q',[],'e',[],'grf',[],'grf3',[],'r',[], ...
                    'qraw',[],'link',[],'dep',[],'cir',[],'cirG',[],'copR',[],'copL',[],'kneeG',[], ...
                    'terms',[],'steps',zeros(0,4)); end
            obs = this.observe(s);
        end

        function [obs, r, done, info] = step(this, action)
            c = this.cfg; s = this.state; m = this.model;
            a = max(-1, min(1, double(action(:))));
            u = c.actionFilter * this.u_prev + (1 - c.actionFilter) * a;
            ex = min(1, max(c.minExcitation, 0.5 * (u + 1)));
            for i = 1:numel(ex), this.ctrlFns{this.actIdx(i)}.setValue(ex(i)); end
            failed = false;
            try
                y0 = s.getY().getAsMat();
                opts = odeset('RelTol', c.odeRelTol, 'AbsTol', c.odeAbsTol, 'Refine', 1);
                [~, Y] = c.odeSolver(@(tt, yy) this.dynamics(s, tt, yy), [this.t, this.t + c.dt], y0, opts);
                s.setTime(this.t + c.dt);
                s.setY(org.opensim.modeling.Vector.createFromMat(Y(end,:)'));
                this.projectFourBar(s);
                m.realizeDynamics(s);
                if any(~isfinite(Y(end,:))), failed = true; end
            catch
                failed = true;
            end
            this.t = this.t + c.dt; this.k = this.k + 1; this.state = s;
            stepEv = zeros(1,4);
            try
                obs = this.observe(s);
                [r, terms, stepEv] = this.reward(s, ex, u);
                ty = this.coords.pelvis_ty.getValue(s);
                tilt = this.coords.pelvis_tilt.getValue(s);
                fell = ty < c.minPelvisHeight || abs(tilt) > c.maxTilt;
            catch
                obs = zeros(this.getObservationInfo().Dimension); r = 0; fell = true; failed = true;
                terms = FreeGaitEnv.emptyTerms();
            end
            if failed || any(~isfinite(obs)), fell = true; obs(~isfinite(obs)) = 0; end
            if fell, r = r - c.fallPenalty; end
            done = fell || this.k >= c.maxSteps;
            this.u_prev = u; this.e_prev = ex;
            info = terms;
            if this.record
                this.traj.t(end+1,1) = this.t;
                this.traj.q(end+1,:) = this.coordVector(s);
                this.traj.e(end+1,:) = ex';
                this.traj.grf3(end+1,:) = this.grf3(s);
                this.traj.grf(end+1,:) = this.traj.grf3(end, [2 5]);
                this.traj.r(end+1,1) = r;
                this.traj.terms(end+1,:) = struct2array(terms);
                if stepEv(1) > 0, this.traj.steps(end+1,:) = stepEv; end
                this.recordFourBar(s);
            end
        end

        function yd = dynamics(this, s, t, y)
            s.setTime(t);
            s.setY(org.opensim.modeling.Vector.createFromMat(y));
            this.model.realizeAcceleration(s);
            yd = s.getYDot().getAsMat();
        end
    end

    methods (Access = private)
        function ok = initPose(this, s, i, strict)
            % estado i do banco (marchas simuladas da etapa 3) + pequeno ruido
            c = this.cfg; m = this.model; ok = true; P = this.pool;
            jit = c.initJitter * (this.fixedInit == 0);
            for j = 1:numel(c.allCoords)
                n = c.allCoords{j}; cj = this.coords.(n);
                if ismember(n, c.lockedCoords), continue, end
                q = P.q(i,j); qd = P.qd(i,j);
                if ~ismember(n, {'pelvis_tx','pelvis_ty'}), q = q + jit * randn; qd = qd + 10 * jit * randn; end
                cj.setValue(s, q, false); cj.setSpeedValue(s, qd);
            end
            this.coords.pelvis_tx.setValue(s, 0, false);
            this.projectFourBar(s);
            this.coords.pelvis_ty.setValue(s, 1.0, false);
            m.realizePosition(s);
            this.coords.pelvis_ty.setValue(s, 1.0 - this.lowestContact(s) - c.initPenetration, true);
            this.projectFourBar(s);
            for k = 1:numel(this.muscles)
                this.muscles{k}.setActivation(s, max(0.02, P.e(i,k)));
            end
            try
                m.equilibrateMuscles(s);
            catch
                ok = ~strict;
            end
        end

        function h = lowestContact(this, s)
            import org.opensim.modeling.*
            bs = this.model.getBodySet();
            spheres = {'foot_r',[-0.03 -0.005 0],0.05; 'foot_r',[0.15 -0.03 -0.026],0.025; ...
                       'foot_r',[0.15 -0.03 0.026],0.025; 'calcn_l',[0.03 0.02 0],0.05; ...
                       'toes_l',[0.02 -0.005 -0.026],0.025; 'toes_l',[0.02 -0.005 0.026],0.025};
            h = inf;
            for i = 1:size(spheres,1)
                p = bs.get(spheres{i,1}).findStationLocationInGround(s, Vec3(spheres{i,2}(1), spheres{i,2}(2), spheres{i,2}(3)));
                h = min(h, p.get(1) - spheres{i,3});
            end
        end

        function g = grf3(this, s)
            % forca de reacao do solo SOBRE cada pe (N): [Fx Fy Fz]_D [Fx Fy Fz]_E
            this.model.realizeDynamics(s);
            g = zeros(1, 6);
            for i = 1:2
                v = this.contactForces{i}.getRecordValues(s);
                g(3*i-2:3*i) = -[v.get(0) v.get(1) v.get(2)];
            end
        end

        function x = heelX(this, s, i)
            p = this.heelBody{i}.getPositionInGround(s); x = p.get(0);
        end

        function obs = observe(this, s)
            c = this.cfg; q = []; qd = [];
            for n = c.obsCoords
                [qv, qdv] = this.jointQ(s, n{1});
                if ~strcmp(n{1}, 'pelvis_tx'), q(end+1,1) = qv; end %#ok<AGROW>
                qd(end+1,1) = qdv; %#ok<AGROW>
            end
            act = zeros(numel(this.muscles),1);
            for i = 1:numel(this.muscles), act(i) = this.muscles{i}.getActivation(s); end
            g3 = this.grf3(s); g = max(0, [g3(2); g3(5)]) / this.bodyWeight;
            if c.clock, ph = 2*pi*this.t / c.clockPeriod; clk = [sin(ph); cos(ph)]; else, clk = [0; 0]; end
            obs = [q; 0.1*qd; act; g; clk; zeros(c.nRefSlots,1); this.u_prev];
        end

        function [r, terms, stepEv] = reward(this, s, ex, u)
            c = this.cfg; stepEv = zeros(1,4);
            vx = this.coords.pelvis_tx.getSpeedValue(s);
            tilt = this.coords.pelvis_tilt.getValue(s);
            tiltd = this.coords.pelvis_tilt.getSpeedValue(s);
            ty = this.coords.pelvis_ty.getValue(s);
            % velocidade de caminhada: 1 dentro da faixa [vLo, vHi]
            dv = max([0, c.vLo - vx, vx - c.vHi]);
            terms.speed = exp(-c.kSpeed * dv^2);
            terms.upright = exp(-c.kUpright * tilt^2 - c.kTiltRate * tiltd^2 - c.kHeight * max(0, c.heightRef - ty)^2);
            % contatos e passos alternados
            g3 = this.grf3(s); gv = [g3(2) g3(5)];
            on = gv >= c.grfThr; terms.step = 0;
            for i = 1:2
                if on(i) && ~this.contact(i)          % contato inicial do pe i
                    o = 3 - i;
                    len = this.heelX(s, i) - this.heelX(s, o);
                    if this.lastStrike == o && len > c.minStepLen
                        T = this.t - this.lastStrikeT(o);        % tempo deste passo
                        this.stepT(i) = T; To = this.stepT(o);
                        if isfinite(To), si = (T - To) / (0.5 * (T + To)); sym = 0.5 + 0.5 * exp(-0.5 * (si / c.symWidth)^2);
                        else, sym = 0.75; end
                        terms.step = c.wStep * min(len, c.stepLenRef) / c.stepLenRef * sym;
                        this.nSteps = this.nSteps + 1;
                        stepEv = [i, this.t, len, T];
                    end
                    this.lastStrike = i; this.lastStrikeT(i) = this.t;
                end
            end
            this.contact = on;
            % penalidades fisiologicas
            terms.effort = sum(this.wF .* ex.^2);                  % ~ custo metabolico
            terms.smooth = mean((ex - this.e_prev).^2);
            terms.rom = 0;
            for n = fieldnames(c.rom)'
                q = this.jointQ(s, n{1}); lim = deg2rad(c.rom.(n{1}));
                terms.rom = terms.rom + max(0, lim(1) - q)^2 + max(0, q - lim(2))^2;
            end
            terms.impact = sum(max(0, gv / this.bodyWeight - c.impactBW).^2);
            terms.flight = double(~any(on));
            task = c.wAlive + c.wSpeed * terms.speed + c.wUpright * terms.upright + terms.step;
            pen = c.wEffort * terms.effort + c.wSmooth * terms.smooth + c.wROM * terms.rom + ...
                  c.wImpact * terms.impact + c.wFlight * terms.flight + c.wAction * mean(u.^2);
            r = task * exp(-pen);
            terms.vx = vx;
        end

        function v = coordVector(this, s)
            n = this.cfg.allCoords; v = zeros(1, numel(n));
            for i = 1:numel(n), v(i) = this.jointQ(s, n{i}); end
        end

        function [q, qd] = jointQ(this, s, name)
            cj = this.coords.(name); q = cj.getValue(s);
            if nargout > 1, qd = cj.getSpeedValue(s); end
            if strcmp(name, this.cfg.fourBar.linkCoord)
                lv = q; q = this.fb.fFlex(lv);
                if nargout > 1, qd = this.fb.fDflex(lv) * qd; end
            end
        end

        function projectFourBar(this, s)
            lk = this.coords.(this.cfg.fourBar.linkCoord);
            lv = lk.getValue(s); lw = lk.getSpeedValue(s);
            for j = 1:2
                this.depC{j}.setValue(s, this.fb.fDep{j}(lv), false);
                this.depC{j}.setSpeedValue(s, this.fb.fDdep{j}(lv) * lw);
            end
        end

        function recordFourBar(this, s)
            import org.opensim.modeling.*
            m = this.model; m.realizeDynamics(s);
            lv = this.coords.(this.cfg.fourBar.linkCoord).getValue(s);
            cirT = this.fb.fCir(lv);
            tf = m.getBodySet().get('transfemur');
            pG = tf.findStationLocationInGround(s, Vec3(cirT(1), cirT(2), 0));
            kn = m.getJointSet().get('4x1_prox').getParentFrame().getPositionInGround(s);
            this.traj.qraw(end+1,:) = cellfun(@(n) this.coords.(n).getValue(s), this.cfg.allCoords);
            this.traj.link(end+1,1) = lv;
            this.traj.dep(end+1,:) = [this.depC{1}.getValue(s) this.depC{2}.getValue(s)];
            this.traj.cir(end+1,:) = cirT;
            this.traj.cirG(end+1,:) = [pG.get(0) pG.get(1) pG.get(2)];
            this.traj.kneeG(end+1,:) = [kn.get(0) kn.get(1) kn.get(2)];
            this.traj.copR(end+1,:) = this.cop(s, 1);
            this.traj.copL(end+1,:) = this.cop(s, 2);
        end

        function p = cop(this, s, i)
            v = this.contactForces{i}.getRecordValues(s);
            Fg = [v.get(0) v.get(1) v.get(2)]; Tg = [v.get(3) v.get(4) v.get(5)];
            if abs(Fg(2)) < 20, p = [NaN NaN NaN]; return, end
            p = [Tg(3) / Fg(2), 0, -Tg(1) / Fg(2)];
        end
    end

    methods (Static)
        function n = obsSize(cfg)
            % igual a etapa 3 (para reaproveitar as redes)
            n = (numel(cfg.obsCoords) - 1) + numel(cfg.obsCoords) + numel(cfg.muscles) ...
                + 2 + 2 + cfg.nRefSlots + numel(cfg.muscles);
        end
        function t = emptyTerms()
            t = struct('speed',0,'upright',0,'step',0,'effort',0,'smooth',0,'rom',0,'impact',0,'flight',0,'vx',0);
        end
    end
end
