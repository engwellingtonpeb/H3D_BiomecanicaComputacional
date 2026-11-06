# RL_OSim: marcha de amputado transfemoral com RL e dinâmica direta OpenSim/MATLAB

As duas etapas compartilham:
- `OpenSIM/`: o modelo `OS4_gait14dof15musc_2Act_RTFP_DynAct.osim` e a geometria;
- `data/wbds/`: o dataset de Fukuchi et al. (2018, CC BY 4.0).

| Pasta | Etapa |
|---|---|
| `etapa1_FD_prothesis_Literature/` | Referência de marcha da literatura. O agente SAC aplica um resíduo sobre o controlador-base. Contém o relatório técnico (`docs/`), os agentes e os resultados da etapa 1. |
| `etapa2_FD_prothesis_Dataset/` | Referência e variabilidade (± 1 DP) medidas no dataset. O agente passa a comandar diretamente os 15 músculos, com o controlador-base retirado por currículo. |
