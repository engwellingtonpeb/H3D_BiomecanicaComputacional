# Etapa 2: marcha guiada pelo dataset

Nesta etapa um agente SAC comanda as excitações dos **15 músculos** do modelo. O objetivo é que a marcha fique o mais próxima possível da marcha saudável **medida** no dataset de Fukuchi, Fukuchi & Duarte (2018, *PeerJ* 6:e4640, CC BY 4.0). A comparação usa a média de 42 adultos, e a tolerância é a variabilidade entre eles (± 1 DP).

**Mantido da etapa 1:**
- o modelo `../OpenSIM/OS4_gait14dof15musc_2Act_RTFP_DynAct.osim`;
- a dinâmica direta: o OpenSim avalia ẏ = f(t, y, u) pela API Java, e o `ode23` do MATLAB integra, com segurador de ordem zero (ZOH) de 10 ms;
- a prótese passiva: mola + amortecedor no joelho e no tornozelo.

## O que muda em relação à etapa 1

| | Etapa 1 | Etapa 2 |
|---|---|---|
| Referência | Curvas da literatura (Winter/Perry), 1,2 m/s, ciclo de 1,1 s | Dataset, classe lenta: 0,86 m/s, passada de 1,20 s, apoio de 65 % |
| Ação | Resíduo de ±0,25 sobre o controlador-base | e = β(e_base + 0,25u) + (1−β)(u+1)/2, com β levado de 1 a 0 |
| Pose na recompensa | Erro em rad² | Erro em unidades de DP entre sujeitos (z), com piso de 2° |
| Forças de reação do solo | Não entram | Vertical e ântero-posterior de cada pé, em z |

### Currículo de β

O β começa em 1 e cai 0,1 sempre que a avaliação determinística anda 10 s. Se a avaliação cair antes de 5 s, ele sobe 0,05. Com **β = 0 não há mais controlador-base**: o agente define sozinho a excitação de cada músculo.

## Resultados (120 blocos de 40 episódios)

O β chegou a 0 no bloco 86. Depois disso, o currículo oscilou entre 0 e 0,15 até o fim, e o treino terminou com β = 0,15. O `finalize_ds.m` reavaliou todos os agentes que tinham andado 10 s com β = 0. Cada um rodou 20 s, partindo de 0% e de 50% do ciclo. Todos andaram os 20 s, e o melhor foi o do **bloco 86** (`agents/best_agent_final.mat`):

| Agente (β = 0) | v (m/s) | Score z | Ciclo dentro de ± 1 DP | Apoio D/E |
|---|---|---|---|---|
| **bloco 86** | 0,77 | **5,51** | **31 %** | 63 / 99 % |
| bloco 106 | 0,71 | 5,65 | 25 % | 63 / 92 % |
| bloco 104 | 0,56 | 6,25 | 20 % | 75 / 94 % |
| bloco 102 | 0,55 | 6,33 | 21 % | 70 / 99 % |

O apoio de referência no dataset é 65 % do ciclo. A tabela completa está em `results/agent_selection.csv`, e o vídeo em [`results/final_p50_opensim.mp4`](results/final_p50_opensim.mp4).

**Limitação principal:** a perna intacta (E) fica no chão por 92–99 % do ciclo, contra 65 % na marcha saudável. Já a prótese tem apoio próximo do fisiológico.

**Próximos passos:**
- recompensa explícita de tempo de apoio e de balanço por pé;
- currículo de β mais lento: passos de 0,05, com 2 avaliações boas seguidas antes de baixar.

## Pastas

| Pasta | Conteúdo |
|---|---|
| `src/` | `datasetReference.m` (referência e DP), `gaitConfigDS.m`, `DatasetGaitEnv.m`, `evaluate_ds.m`, `train_ds.m` |
| `data/` | `ref_S.mat` e `ref_C.mat`, as referências em cache |
| `agents/` | `sac_checkpoint.mat`, `agent_chunkNNN_bXXX.mat` (agentes que andaram 10 s com β = XXX/100) e `best_agent_direct.mat` (melhor com β = 0) |
| `results/` | Trajetórias das avaliações (`*_traj.mat`, `*_motion.mot`) |
| `logs/` | `train_console.log` e `train_log.csv` |

As colunas do `train_log.csv` são: bloco, β, R médio, R máximo, passos médios, passos máximos, tempo (s), sobrevida na avaliação (s), v (m/s), score (z médio), % dentro de 1 DP, apoio direito (%) e apoio esquerdo (%).

## Uso

```matlab
cd etapa2_FD_prothesis_Dataset/src
train_ds('nChunks',120,'workers',4)          % retoma sozinho do checkpoint
M = evaluate_ds(load('../agents/best_agent_direct.mat').agent, 'baseWeight', 0, 'tag', 'final');
```

As redes iniciais vêm do melhor agente da etapa 1, que é apenas lido. Para treinar do zero: `'init','scratch'`.
