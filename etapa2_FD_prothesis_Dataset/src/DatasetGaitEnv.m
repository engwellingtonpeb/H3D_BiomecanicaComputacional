classdef DatasetGaitEnv < rl.env.MATLABEnvironment
    %DATASETGAITENV ETAPA 2. Mesmo modelo (OS4_gait14dof15musc_2Act_RTFP_DynAct.osim)
    %   e mesma dinamica direta da etapa 1 (OpenSim avalia ydot, ode23 do
    %   MATLAB integra, ZOH de 10 ms).
    %
    %   Acao (15) -> excitacoes de TODOS os musculos do modelo:
    %       e = beta*(e_base + 0.25*u) + (1-beta)*(u+1)/2
    %   com beta (cfg.baseWeight) levado de 1 a 0 pelo curriculo: no fim o
    %   agente comanda sozinho cada musculo.
    %   Referencia: media e DP entre sujeitos do dataset de Fukuchi et al.
    %   (2018); a recompensa mede os erros em unidades de DP (angulos e GRF).
    %   Protese PASSIVA: joelho e tornozelo apenas com mola + amortecedor.

    properties
        cfg
        ref
        assist = 1          % nivel atual da assistencia na pelve (0..1)
        episodeCount = 0
        record = false      % gravar trajetoria (avaliacao)
        traj = struct()
    end

    properties (Transient)
        model; state; ctrlFns; coords; muscles; contactForces
        actIdx; assistIdx
        built = false
    end

    properties (Access = private)
        t = 0; k = 0; phase0 = 0
        u_prev; e_prev
        R            % matriz de bracos de momento (nJoint x nMus)
        ty0 = 0.94
        bodyWeight = 1
    end

    methods
        function this = DatasetGaitEnv(cfg)
            if nargin < 1, cfg = gaitConfigDS(); end
            nObs = DatasetGaitEnv.obsSize(cfg);
            oi = rlNumericSpec([nObs 1]);
            oi.Name = 'gait_obs';
            ai = rlNumericSpec([numel(cfg.muscles) 1], 'LowerLimit', -1, 'UpperLimit', 1);
            ai.Name = 'muscle_excitations';
            this = this@rl.env.MATLABEnvironment(oi, ai);
            % referencia carregada aqui (no cliente) e enviada aos workers junto com o ambiente
            this.ref = datasetReference('class', cfg.refClass);
            cfg.targetSpeed = this.ref.speed;
            this.cfg = cfg;
        end

        %% ---------------- construcao do modelo ----------------
        function build(this)
            import org.opensim.modeling.*
            c = this.cfg;
            m = Model(c.modelFile);
            m.setUseVisualizer(false);
            cs = m.getCoordinateSet();
            for n = c.lockedCoords
                cs.get(n{1}).setDefaultLocked(true);
                cs.get(n{1}).setDefaultValue(0);
            end
            % PROTESE PASSIVA: desliga os atuadores ativos do joelho/tornozelo
            % e adiciona mola + amortecedor em cada junta protetica
            % (os batentes KneeLimit_r / AnkleLimit_r do modelo continuam).
            for n = {'knee_actuator','ankle_actuator'}
                m.updForceSet().get(n{1}).set_appliesForce(false);
            end
            P = c.prosthesis;
            for n = fieldnames(P)'
                pp = P.(n{1});
                sp = SpringGeneralizedForce(pp.coord);
                sp.setName(['passive_' n{1}]);
                sp.setStiffness(pp.stiffness); sp.setRestLength(pp.rest); sp.setViscosity(pp.damping);
                m.addForce(sp);
            end
            % atuadores de assistencia na pelve ("mao de Deus", curriculo)
            for n = c.assistCoords
                ca = CoordinateActuator(n{1});
                ca.setName(['assist_' n{1}]);
                ca.setOptimalForce(1);
                ca.setMinControl(-1e5); ca.setMaxControl(1e5);
                m.addForce(ca);
            end
            brain = PrescribedController();
            acts = m.getActuators();
            nA = acts.getSize();
            names = cell(nA,1);
            for i = 0:nA-1
                a = acts.get(i);
                names{i+1} = char(a.getName());
                brain.addActuator(a);
                brain.prescribeControlForActuator(names{i+1}, Constant(0));
            end
            m.addController(brain);
            s = m.initSystem();

            fs = brain.get_ControlFunctions();
            this.ctrlFns = cell(nA,1);
            for i = 0:nA-1, this.ctrlFns{i+1} = Constant.safeDownCast(fs.get(i)); end
            % mapeamento acao -> indice de atuador
            order = c.muscles;
            this.actIdx = cellfun(@(x) find(strcmp(names, x)), order);
            this.assistIdx = cellfun(@(x) find(strcmp(names, ['assist_' x])), c.assistCoords);

            this.coords = struct();
            for n = c.allCoords
                this.coords.(n{1}) = cs.get(n{1});
            end
            ms = m.getMuscles();
            this.muscles = cell(numel(c.muscles),1);
            for i = 1:numel(c.muscles), this.muscles{i} = ms.get(c.muscles{i}); end
            this.contactForces = {HuntCrossleyForce.safeDownCast(m.getForceSet().get('foot_r')), ...
                                  HuntCrossleyForce.safeDownCast(m.getForceSet().get('foot_l'))};
            this.bodyWeight = m.getTotalMass(s) * 9.80665;
            this.model = m; this.state = s;
            this.built = true;
        end

        %% ---------------- reset ----------------
        function obs = reset(this)
            if ~this.built, this.build(); end
            c = this.cfg; m = this.model;
            s = m.initializeState();
            this.episodeCount = this.episodeCount + 1;
            if c.assistDecayEpisodes > 0
                this.assist = c.assistStart * max(0, 1 - this.episodeCount / c.assistDecayEpisodes);
            else
                this.assist = c.assistStart;
            end
            % Inicializacao por estado de referencia (RSI). Se o equilibrio
            % musculo-tendao nao convergir nessa fase, sorteia outra.
            for attempt = 1:6
                if c.randomInitPhase || attempt > 1, this.phase0 = rand; else, this.phase0 = c.initPhase; end
                s = m.initializeState();
                if this.initPose(s, attempt < 6), break; end
            end
            this.state = s; this.t = 0; this.k = 0;
            this.u_prev = zeros(numel(c.muscles),1);
            this.e_prev = 0.05*ones(numel(c.muscles),1);
            this.R = this.momentArms(s);
            if this.record, this.traj = struct('t',[],'q',[],'e',[],'grf',[],'grf3',[],'r',[],'assist',[]); end
            obs = this.observe(s);
        end
    end

    methods (Access = private)
        function ok = initPose(this, s, strict)
            % coloca o modelo na pose de referencia da fase this.phase0;
            % ok = false se o equilibrio dos musculos falhar (strict)
            c = this.cfg; m = this.model; ok = true;
            [q, qd] = this.refAt(this.phase0);
            jn = fieldnames(q);
            for i = 1:numel(jn)
                this.coords.(jn{i}).setValue(s, q.(jn{i}), false);
                this.coords.(jn{i}).setSpeedValue(s, qd.(jn{i}));
            end
            this.coords.pelvis_tilt.setValue(s, c.pelvisTiltRef, false);
            this.coords.pelvis_tx.setValue(s, 0, false);
            this.coords.pelvis_tx.setSpeedValue(s, c.targetSpeed);
            % altura da pelve: pe mais baixo apenas tocando o solo
            this.coords.pelvis_ty.setValue(s, 1.0, false);
            m.realizePosition(s);
            this.coords.pelvis_ty.setValue(s, 1.0 - this.lowestContact(s) - c.initPenetration, true);
            this.ty0 = this.coords.pelvis_ty.getValue(s);
            % ativacoes iniciais e equilibrio musculo-tendao
            for i = 1:numel(this.muscles)
                this.muscles{i}.setActivation(s, 0.05);
            end
            try
                m.equilibrateMuscles(s);
            catch
                % ultimo recurso: segue com comprimentos de fibra padrao
                ok = ~strict;
            end
        end
    end

    methods

        %% ---------------- step ----------------
        function [obs, r, done, info] = step(this, action)
            c = this.cfg; s = this.state; m = this.model;
            a = max(-1, min(1, double(action(:))));
            % filtro passa-baixa nas acoes -> excitacoes mais suaves
            u = c.actionFilter * this.u_prev + (1 - c.actionFilter) * a;
            phase = this.currentPhase();
            b = c.baseWeight;
            if b > 0, eBase = this.baseline(s, phase); else, eBase = zeros(size(u)); end
            ex = min(1, max(0.01, b * (eBase + c.residualScale * u) + (1 - b) * 0.5 * (u + 1)));
            for i = 1:numel(ex), this.ctrlFns{this.actIdx(i)}.setValue(ex(i)); end
            aF = this.assistForces(s, phase);
            for i = 1:numel(aF), this.ctrlFns{this.assistIdx(i)}.setValue(aF(i)); end

            failed = false;
            try
                % Integracao da ODE inteiramente no MATLAB. O OpenSim so
                % avalia ydot = f(t, y, u) (u constante no intervalo: ZOH).
                y0 = s.getY().getAsMat();
                opts = odeset('RelTol', c.odeRelTol, 'AbsTol', c.odeAbsTol, 'Refine', 1);
                [~, Y] = c.odeSolver(@(tt, yy) this.dynamics(s, tt, yy), [this.t, this.t + c.dt], y0, opts);
                s.setTime(this.t + c.dt);
                s.setY(org.opensim.modeling.Vector.createFromMat(Y(end,:)'));
                m.realizeDynamics(s);
                if any(~isfinite(Y(end,:))), failed = true; end
            catch
                failed = true;
            end
            this.t = this.t + c.dt; this.k = this.k + 1;
            this.state = s;
            if mod(this.k, c.momentArmEvery) == 0 && ~failed
                this.R = this.momentArms(s);
            end

            try
                obs = this.observe(s);
                [r, terms] = this.reward(s, ex, u, aF);
                ty = this.coords.pelvis_ty.getValue(s);
                tilt = this.coords.pelvis_tilt.getValue(s);
                fell = ty < c.minPelvisHeight || abs(tilt - c.pelvisTiltRef) > c.maxTilt;
            catch
                % estado invalido (ex.: integracao divergiu): encerra o episodio
                obs = zeros(this.getObservationInfo().Dimension); r = 0; fell = true; failed = true;
                terms = struct('pose',0,'vel',0,'grf',0,'speed',0,'upright',0,'effort',0,'smooth',0,'assist',0);
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
                this.traj.grf(end+1,:) = this.grf(s)';
                this.traj.grf3(end+1,:) = this.grf3(s);   % [Fx Fy Fz]_dir [Fx Fy Fz]_esq (N)
                this.traj.r(end+1,1) = r;
                this.traj.assist(end+1,:) = aF';
            end
        end
    end

    methods
        function yd = dynamics(this, s, t, y)
            %DYNAMICS Lado direito da ODE: y = [q; u; z] (Simbody), z =
            %ativacoes e comprimentos de fibra dos musculos + ativacao dos
            %atuadores da protese. Avaliado pelo OpenSim, integrado no MATLAB.
            s.setTime(t);
            s.setY(org.opensim.modeling.Vector.createFromMat(y));
            this.model.realizeAcceleration(s);
            yd = s.getYDot().getAsMat();
        end
    end

    methods (Access = private)
        function ph = currentPhase(this)
            ph = mod(this.phase0 + this.t / this.ref.cycleTime, 1);
        end

        function [gm, gs] = grfRefAt(this, phase)
            % GRF de referencia (BW) e DP: [vert_R vert_L ap_R ap_L]
            R_ = this.ref; L = phase; Rr = mod(phase + 0.5, 1);
            gm = [interpPeriodic(R_.grfv, Rr) interpPeriodic(R_.grfv, L) interpPeriodic(R_.grfap, Rr) interpPeriodic(R_.grfap, L)];
            gs = [interpPeriodic(R_.grfvSD, Rr) interpPeriodic(R_.grfvSD, L) interpPeriodic(R_.grfapSD, Rr) interpPeriodic(R_.grfapSD, L)];
        end

        function [q, qd, sd] = refAt(this, phase)
            % referencia para a perna esquerda (intacta) na fase phase e
            % para a direita (protese) defasada de meio ciclo; sd = DP entre sujeitos
            R_ = this.ref; T = R_.cycleTime;
            L = phase; Rr = mod(phase + 0.5, 1);
            f = @(sig, p) interpPeriodic(sig, p);
            df = @(sig, p) (interpPeriodic(sig, mod(p+1e-3,1)) - interpPeriodic(sig, mod(p-1e-3,1))) / (2e-3*T);
            if nargout > 2
                sd.hip_flexion_l = f(R_.hipSD, L);   sd.hip_flexion_r = f(R_.hipSD, Rr);
                sd.knee_angle_l  = f(R_.kneeSD, L);  sd.knee_angle_r  = f(R_.kneeSD, Rr);
                sd.ankle_angle_l = f(R_.ankleSD, L); sd.ankle_angle_r = f(R_.ankleSD, Rr);
                sd.hip_adduction_l = f(R_.hipAddSD, L); sd.hip_adduction_r = f(R_.hipAddSD, Rr);
            end
            q.hip_flexion_l = f(R_.hip, L);    qd.hip_flexion_l = df(R_.hip, L);
            q.knee_angle_l  = f(R_.knee, L);   qd.knee_angle_l  = df(R_.knee, L);
            q.ankle_angle_l = f(R_.ankle, L);  qd.ankle_angle_l = df(R_.ankle, L);
            q.hip_flexion_r = f(R_.hip, Rr);   qd.hip_flexion_r = df(R_.hip, Rr);
            q.knee_angle_r  = f(R_.knee, Rr);  qd.knee_angle_r  = df(R_.knee, Rr);
            q.ankle_angle_r = f(R_.ankle, Rr); qd.ankle_angle_r = df(R_.ankle, Rr);
            q.hip_adduction_l = f(R_.hipAdd, L);  qd.hip_adduction_l = df(R_.hipAdd, L);
            q.hip_adduction_r = f(R_.hipAdd, Rr); qd.hip_adduction_r = df(R_.hipAdd, Rr);
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

        function R = momentArms(this, s)
            jn = this.cfg.ctrlJoints;
            R = zeros(numel(jn), numel(this.muscles));
            this.model.realizePosition(s);
            for j = 1:numel(jn)
                cj = this.coords.(jn{j});
                for i = 1:numel(this.muscles)
                    if this.cfg.muscleJointMask(j,i)
                        R(j,i) = this.muscles{i}.computeMomentArm(s, cj);
                    end
                end
            end
        end

        function e = baseline(this, s, phase)
            %Controlador-base: PD no espaco articular -> torques desejados ->
            %excitacoes por minimos quadrados nao-negativos (tipo otimizacao
            %estatica). So musculos; a protese e passiva.
            c = this.cfg;
            [qr, qdr] = this.refAt(mod(phase + c.dt/this.ref.cycleTime, 1));
            jn = c.ctrlJoints; nJ = numel(jn);
            tau = zeros(nJ,1);
            tilt = this.coords.pelvis_tilt.getValue(s) - c.pelvisTiltRef;
            tiltd = this.coords.pelvis_tilt.getSpeedValue(s);
            g = this.grf(s) / this.bodyWeight;     % [vert_r; vert_l]
            for j = 1:nJ
                cj = this.coords.(jn{j});
                tau(j) = c.Kp(j) * (qr.(jn{j}) - cj.getValue(s)) + c.Kd(j) * (qdr.(jn{j}) - cj.getSpeedValue(s));
            end
            % equilibrio do tronco pelo quadril de apoio (estilo SIMBICON)
            jr = strcmp(jn, 'hip_flexion_r'); jl = strcmp(jn, 'hip_flexion_l');
            wR = g(1) / max(g(1) + g(2), 1e-3); wL = 1 - wR;
            if g(1) + g(2) < 0.05, wR = 0; wL = 0; end
            torso = c.KpTorso * tilt + c.KdTorso * tiltd;
            tau(jr) = tau(jr) + wR * torso;
            tau(jl) = tau(jl) + wL * torso;
            % musculos: min ||F e - tau||^2 + lambda ||e||^2, e >= 0
            F = this.R .* (c.Fmax(:)');
            A = [F; sqrt(c.lambda) * diag(c.Fmax)];
            b = [tau; zeros(numel(c.Fmax),1)];
            e = lsqnonneg(A, b);
            e = min(1, e + c.baseExcitation);
            % protese passiva: nenhum controle sobre joelho/tornozelo direitos
        end

        function f = assistForces(this, s, phase)
            c = this.cfg; f = zeros(numel(c.assistCoords),1);
            if this.assist <= 0, return; end
            for i = 1:numel(c.assistCoords)
                n = c.assistCoords{i}; cj = this.coords.(n);
                switch n
                    case 'pelvis_tx'
                        f(i) = c.assistKd(i) * (c.targetSpeed - cj.getSpeedValue(s));
                    case 'pelvis_ty'
                        f(i) = max(0, c.assistKp(i) * (this.ty0 - cj.getValue(s)) - c.assistKd(i) * cj.getSpeedValue(s));
                    case 'pelvis_tilt'
                        f(i) = c.assistKp(i) * (c.pelvisTiltRef - cj.getValue(s)) - c.assistKd(i) * cj.getSpeedValue(s);
                    otherwise
                        f(i) = -c.assistKp(i) * cj.getValue(s) - c.assistKd(i) * cj.getSpeedValue(s);
                end
            end
            f = this.assist * f;
        end

        function g = grf(this, s)
            this.model.realizeDynamics(s);
            g = zeros(2,1);
            for i = 1:2
                v = this.contactForces{i}.getRecordValues(s);
                g(i) = -v.get(1);    % componente vertical sobre o pe
            end
            g = max(0, g);
        end

        function g = grf3(this, s)
            % forca de reacao do solo SOBRE cada pe (N), eixos do ground:
            % X antero-posterior (+ para frente), Y vertical, Z medio-lateral
            g = zeros(1, 6);
            for i = 1:2
                v = this.contactForces{i}.getRecordValues(s);   % 0..2 = forca sobre o solo
                g(3*i-2:3*i) = -[v.get(0) v.get(1) v.get(2)];
            end
        end

        function obs = observe(this, s)
            c = this.cfg;
            q = []; qd = [];
            for n = c.obsCoords
                cj = this.coords.(n{1});
                if ~strcmp(n{1}, 'pelvis_tx'), q(end+1,1) = cj.getValue(s); end %#ok<AGROW>
                qd(end+1,1) = cj.getSpeedValue(s); %#ok<AGROW>
            end
            act = zeros(numel(this.muscles),1);
            for i = 1:numel(this.muscles), act(i) = this.muscles{i}.getActivation(s); end
            ph = this.currentPhase();
            [qr, ~] = this.refAt(mod(ph + c.dt/this.ref.cycleTime, 1));
            qref = cellfun(@(n) qr.(n), c.trackJoints)';
            g = this.grf(s) / this.bodyWeight;
            obs = [q; 0.1*qd; act; g; sin(2*pi*ph); cos(2*pi*ph); qref; this.u_prev];
        end

        function [r, terms] = reward(this, s, ex, u, aF)
            c = this.cfg; ph = this.currentPhase();
            [qr, qdr, qsd] = this.refAt(ph);
            % pose: erro de cada junta em unidades de DP entre sujeitos
            ez = 0; ev = 0; sdMin = deg2rad(c.sdFloorDeg);
            for j = 1:numel(c.trackJoints)
                n = c.trackJoints{j}; cj = this.coords.(n);
                ez = ez + c.trackW(j) * ((cj.getValue(s) - qr.(n)) / max(qsd.(n), sdMin))^2;
                ev = ev + c.trackW(j) * (cj.getSpeedValue(s) - qdr.(n))^2;
            end
            ez = ez / sum(c.trackW);
            % GRF de cada pe (vertical e antero-posterior) contra o dataset
            [gm, gs] = this.grfRefAt(ph);                 % [vR vL apR apL]
            g3 = this.grf3(s) / this.bodyWeight;          % [Fx Fy Fz]_R [Fx Fy Fz]_L
            gsim = [g3(2) g3(5) g3(1) g3(4)];
            gs = max(gs, [c.sdFloorGrfv c.sdFloorGrfv c.sdFloorGrfap c.sdFloorGrfap]);
            eg = sum([1 1 0.5 0.5] .* ((gsim - gm) ./ gs).^2) / 3;
            vx = this.coords.pelvis_tx.getSpeedValue(s);
            tilt = this.coords.pelvis_tilt.getValue(s) - c.pelvisTiltRef;
            ty = this.coords.pelvis_ty.getValue(s);
            terms.pose  = exp(-c.kPose * ez);
            terms.vel   = exp(-c.kVel * ev);
            terms.grf   = exp(-c.kGrf * eg);
            terms.zPose = sqrt(ez); terms.zGrf = sqrt(eg);
            terms.speed = exp(-c.kSpeed * (vx - c.targetSpeed)^2);
            terms.upright = exp(-c.kUpright * tilt^2 - c.kHeight * (ty - this.ty0)^2);
            terms.effort = mean(ex.^3);                        % ~ fadiga muscular
            terms.smooth = mean((ex - this.e_prev).^2);        % variacao de excitacao
            terms.assist = min(1, mean(abs(aF) ./ c.assistNorm(:)));
            % forma multiplicativa: recompensa sempre >= 0, logo cair (perder
            % os passos futuros) e sempre pior do que continuar andando
            task = c.wPose*terms.pose + c.wVel*terms.vel + c.wGrf*terms.grf + c.wSpeed*terms.speed + ...
                   c.wUpright*terms.upright + c.wAlive;
            pen = c.wEffort*terms.effort + c.wSmooth*terms.smooth + ...
                  c.wAssist*terms.assist + c.wAction*mean(u.^2);
            r = task * exp(-pen);
        end

        function v = coordVector(this, s)
            n = this.cfg.allCoords; v = zeros(1, numel(n));
            for i = 1:numel(n), v(i) = this.coords.(n{i}).getValue(s); end
        end
    end

    methods (Static)
        function n = obsSize(cfg)
            n = (numel(cfg.obsCoords) - 1) + numel(cfg.obsCoords) + numel(cfg.muscles) ...
                + 2 + 2 + numel(cfg.trackJoints) + numel(cfg.muscles);
        end
    end
end

function y = interpPeriodic(sig, p)
N = numel(sig); x = p * N; i0 = floor(x); w = x - i0;
i0 = mod(i0, N) + 1; i1 = mod(i0, N) + 1;
y = (1 - w) * sig(i0) + w * sig(i1);
end
