# Marcha com prótese transfemoral: dinâmica direta + RL (MATLAB + OpenSim)

Modelo: `OpenSIM/OS4_gait14dof15musc_2Act_RTFP_DynAct.osim`. É um amputado transfemoral direito com 15 músculos Thelen2003. A **prótese é passiva**: os atuadores `knee_actuator` e `ankle_actuator` do modelo ficam desligados e são trocados por mola + amortecedor (`SpringGeneralizedForce`), mantendo os batentes `KneeLimit_r` e `AnkleLimit_r`. Joelho: k = 5 Nm/rad, b = 3 Nm·s/rad. Tornozelo tipo ESAR: k = 350 Nm/rad, b = 2 Nm·s/rad. Os valores estão em `gaitConfig.m` (`c.prosthesis`).

## Arquitetura

| Parte | Onde |
|---|---|
| Equações do modelo, ẏ = f(t, y, u) | OpenSim (via API Java: `setY` → `realizeAcceleration` → `getYDot`) |
| **Integração da ODE** | **MATLAB** (`ode23`, RelTol 1e-3, AbsTol 1e-4; configurável em `gaitConfig`) |
| Controle (u constante em cada passo de 10 ms, ZOH) | MATLAB: controlador-base + residual do agente RL (só músculos) |
| Aprendizagem por reforço | MATLAB RL Toolbox, agente SAC, treino paralelo |

O vetor de estados tem 66 variáveis: 17 coordenadas q, 17 velocidades u e 32 estados musculares (15 ativações, 15 comprimentos de fibra e 2 ativações dos atuadores desligados, que ficam inertes). O `Manager` do OpenSim não é usado.

### Ação (15 sinais, só músculos)
- 15 excitações musculares: `abd_r, add_r, glut_max_r, iliopsoas_r, abd_l, add_l, hamstrings_l, bifemsh_l, glut_max_l, iliopsoas_l, rect_fem_l, vasti_l, gastroc_l, soleus_l, tib_ant_l`

O agente produz **resíduos** sobre um controlador-base, e as ações passam por um filtro passa-baixa.
- O controlador-base faz PD articular em torno da cinemática saudável de referência. Os torques viram excitações por mínimos quadrados não negativos, como numa otimização estática. O tronco é equilibrado pelo quadril de apoio, no estilo SIMBICON. Joelho e tornozelo protéticos não recebem controle; só podem ser influenciados indiretamente pelos músculos do quadril residual, como num amputado real.

### Recompensa (estilo DeepMimic)
Imitação da cinemática saudável (Winter/Perry), velocidade de 1,2 m/s e postura ereta da pelve. As penalidades (esforço Σe³, variação das excitações, uso da assistência) entram de forma multiplicativa: `r = tarefa · exp(−penalidades)`. Assim r ≥ 0 e cair é sempre pior que continuar andando.

### Currículo
Atuadores de assistência na pelve ("mão de Deus") ajudam no começo e são removidos de forma **adaptativa**: a cada bloco, a assistência cai 0,05 se os episódios duram ≥ 80% do máximo e sobe 0,03 se duram < 50%. A avaliação é sempre **sem** assistência.

Estágio 1 é sagital (`planar=true`): `pelvis_list`, `pelvis_rotation`, `pelvis_tz` e `hip_rotation_r` ficam travados. Para o 3D completo, use `gaitConfig('planar', false)`.

## Arquivos (`src/`)
- `gaitConfig.m`: todos os parâmetros (integrador, ganhos, recompensa, currículo)
- `healthyGaitReference.m`: curvas normativas de quadril, joelho e tornozelo
- `ProstheticGaitEnv.m`: ambiente de RL (`rl.env.MATLABEnvironment`)
- `train_sac.m`: treino em blocos com checkpoint (`results/sac_checkpoint.mat`), avaliação periódica e log em `results/train_log.csv`
- `evaluate_policy.m`: métricas (RMSE e correlação com a referência, simetria de apoio, jerk, velocidade) e saídas `results/<tag>_motion.mot` (abra no OpenSim GUI: *File → Load Motion*) e `results/<tag>_traj.csv`. O PNG só é gerado no MATLAB desktop, porque em `-batch` os gráficos derrubam o MATLAB nesta máquina.

## Uso
```matlab
cd src
train_sac('nChunks', 150, 'episodesPerChunk', 40, 'workers', 4)   % retoma do checkpoint automaticamente
S = load('../results/sac_checkpoint.mat');
M = evaluate_policy(S.agent, 'assist', 0, 'duration', 10, 'tag', 'final')
```

Critério de marcha "fisiologicamente plausível" (`M.plausible`):
- não cai durante a avaliação;
- velocidade entre 0,9 e 1,5 m/s;
- RMSE articular < 10°;
- correlação com a referência > 0,8;
- assimetria de tempo de apoio < 15%.
