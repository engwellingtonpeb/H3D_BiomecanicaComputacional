"""Figuras da etapa 3 (CIR x fase x GRF e assimetria).

Uso: python plot_cir_asym.py <results/cir_asym_dados.mat> <pasta figs>
Os dados vem de compare_cir_asym.m (MATLAB).
"""
import sys
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.io import loadmat

C4 = "#d9541a"   # 4 barras
CP = "#0072bd"   # pino
CD = "#666666"   # dataset


def main(dat, out):
    D = loadmat(dat, squeeze_me=True)
    has4 = bool(D["has4"])
    os.makedirs(out, exist_ok=True)
    g = lambda k: np.atleast_1d(D[k]).astype(float)

    # ---------------- 1) trajetoria do CIR ----------------
    fig, ax = plt.subplots(figsize=(8.5, 7.5))
    cf, cc = g("cent_flex"), np.atleast_2d(D["cent_cir"])
    sel = cf > -100
    ax.plot(cc[sel, 0], cc[sel, 1], "-", color="#bbbbbb", lw=4, label="centroide (0 a 100° de flexao)", zorder=1)
    for fl in (0, 20, 40, 60, 80):
        k = np.argmin(np.abs(cf + fl))
        ax.annotate(f"{fl}°", cc[k], xytext=(6, 0), textcoords="offset points", fontsize=8, color="#555555")
    P = g("piv0")
    ax.plot(P[[0, 2]], P[[1, 3]], "k-", lw=2.5, label="elos em extensao")
    ax.plot(P[[6, 4]], P[[7, 5]], "k-", lw=2.5)
    ax.plot(P[[0, 2, 4, 6]], P[[1, 3, 5, 7]], "o", mfc="w", mec="k", ms=7)
    pin = g("pin")
    ax.plot(pin[0], pin[1], "*", ms=18, mfc=CP, mec="k", label="CIR protese de pino (fixo)", zorder=5)
    if has4:
        ct, ph = np.atleast_2d(D["fb_cirT"]), g("fb_phase")
        sc = ax.scatter(ct[:, 0], ct[:, 1], c=ph, cmap="turbo", s=10, vmin=0, vmax=100,
                        label="CIR 4 barras durante a marcha", zorder=4)
        cb = fig.colorbar(sc, ax=ax)
        cb.set_label("% do ciclo da protese (0 = contato inicial)")
        se = float(D["fb_stanceEnd"])
        cb.ax.axhline(se, color="k", lw=2)
        cb.ax.text(1.6, se, " fim do apoio", va="center", fontsize=8, transform=cb.ax.get_yaxis_transform())
        ax.set_title(f"Trajetoria do CIR da perna em relacao a coxa  |  apoio 0-{se:.0f}%, balanco {se:.0f}-100%")
    else:
        ax.set_title("Centroide do joelho de 4 barras x centro do pino (etapa 3 ainda sem trajetoria)")
    ax.set_xlabel("x anterior (mm) - referencial da coxa")
    ax.set_ylabel("y superior (mm)")
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.1), ncol=2, fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "cir_trajetoria.png"), dpi=150)
    plt.close(fig)

    # ---------------- 2) curvas sincronizadas ----------------
    pct = g("pct")
    panels = [("kneeFlex", "Flexao do joelho\nprotetico (°)", ("ref_knee", "ref_kneeSD")),
              ("cirX", "CIR x anterior\n(mm, coxa)", None),
              ("cirY", "CIR y superior\n(mm, coxa)", None),
              ("grfV", "GRF vertical\n(BW)", ("ref_grfV", "ref_grfVSD")),
              ("grfAP", "GRF antero-post.\n(BW)", ("ref_grfAP", "ref_grfAPSD")),
              ("dcir", "Braco da GRF no CIR\n(mm; >0 extensor)", None)]
    fig, axs = plt.subplots(len(panels), 1, figsize=(10, 13), sharex=True)
    for ax, (v, lab, rf) in zip(axs, panels):
        if rf:
            mu, sd = g(rf[0]), g(rf[1])
            ax.fill_between(pct, mu - sd, mu + sd, color=CD, alpha=0.2, lw=0, label="dataset ±1 DP")
            ax.plot(pct, mu, "--", color=CD, label="dataset (saudavel)")
        for pre, col, name in (("pin", CP, "pino (etapa 2)"), ("fb", C4, "4 barras (etapa 3)")):
            if pre == "fb" and not has4:
                continue
            m, s = g(f"{pre}_{v}"), g(f"{pre}_{v}SD")
            ax.fill_between(pct, m - s, m + s, color=col, alpha=0.18, lw=0)
            ax.plot(pct, m, "-", color=col, lw=2, label=name)
            ax.axvline(float(D[f"{pre}_stanceEnd"]), color=col, ls=":", lw=1.5)
        ax.axvline(float(D["ref_stance"]), color=CD, ls=":", lw=1)
        if v == "dcir":
            ax.axhline(0, color="k", lw=1)
            ax.text(1, ax.get_ylim()[1], " extensor (estavel)", va="top", fontsize=8)
            ax.text(1, ax.get_ylim()[0], " flexor (tende a dobrar)", va="bottom", fontsize=8)
        ax.set_ylabel(lab, fontsize=9)
        ax.grid(alpha=0.3)
        ax.set_xlim(0, 100)
    axs[0].legend(loc="upper left", bbox_to_anchor=(1.01, 1), fontsize=8)
    axs[-1].set_xlabel("% do ciclo da protese (0 = contato inicial do pe protetico; pontilhado = fim do apoio)")
    fig.suptitle("CIR sincronizado com a fase e com a forca de reacao do solo (media ± DP das passadas)")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "cir_fase_grf.png"), dpi=150)
    plt.close(fig)

    # ---------------- 3) assimetria ----------------
    names = [str(x) for x in np.atleast_1d(D["asym_param"])]
    lbl = {"stride": "tempo da passada", "stance": "tempo de apoio", "swing": "tempo de balanco",
           "stancePct": "apoio (% do ciclo)", "stepTime": "tempo do passo", "stepLen": "comprimento do passo",
           "peakV": "pico GRF vertical", "impV": "impulso vertical", "impBrake": "impulso de frenagem",
           "impProp": "impulso propulsivo", "kneeROM": "amplitude do joelho", "hipROM": "amplitude do quadril"}
    y = np.arange(len(names))
    fig, ax = plt.subplots(figsize=(9, 6))
    h = 0.38 if has4 else 0.6
    sp = np.clip(g("asym_pin"), -200, 200)
    ax.barh(y + (h / 2 if has4 else 0), sp, h, color=CP, label="pino (etapa 2)")
    if has4:
        ax.barh(y - h / 2, np.clip(g("asym_fb"), -200, 200), h, color=C4, label="4 barras (etapa 3)")
    ax.axvline(0, color="k", lw=1)
    ax.axvspan(-10, 10, color="#999999", alpha=0.15, label="±10% (aprox. simetrico)")
    ax.set_yticks(y, [lbl.get(n, n) for n in names])
    ax.invert_yaxis()
    ax.set_xlabel("SI = 100·(D − E)/média (%)   |   D = protese, E = perna intacta   (limitado a ±200)")
    ax.set_title("Assimetria entre a perna protetica e a intacta")
    ax.grid(axis="x", alpha=0.3)
    ax.legend(loc="lower right", fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "assimetria.png"), dpi=150)
    plt.close(fig)
    print("figuras em", out)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
