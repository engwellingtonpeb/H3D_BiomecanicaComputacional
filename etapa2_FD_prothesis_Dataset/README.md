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
