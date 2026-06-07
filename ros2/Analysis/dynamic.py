"""
dynamic.py
Análisis de métricas dinámicas de navegación en tiempo real.
Compara Dijkstra vs A* con y sin suavizado durante la ejecución.

Gráficas generadas:
  1-4.   Trayectoria real vs planificada en plano XY
  5-8.   Velocidad lineal y angular a lo largo del tiempo
  9-12.  ETA real vs ETA calculado por Nav2
  13.    Tiempo total de navegación — comparativa entre variantes
  14.    Comparativa de ETA calculado por cada algoritmo
"""

import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# -- Configuración ---------------------------------------------------------------------------
RESULTS_DIR = os.path.join(os.path.dirname(__file__), "results", "dynamic")
PLOTS_DIR   = os.path.join(os.path.dirname(__file__), "plots", "dynamic")
os.makedirs(PLOTS_DIR, exist_ok=True)

# -- Colores y etiquetas ---------------------------------------------------------------------------
COLORS = {
    "dijkstra":          "#0000FF",
    "dijkstra_nosmooth": "#8888FF",
    "astar":             "#FF0000",
    "astar_nosmooth":    "#FF8888",
}

LABELS = {
    "dijkstra":          "Dijkstra (smooth)",
    "dijkstra_nosmooth": "Dijkstra (nosmooth)",
    "astar":             "A* (smooth)",
    "astar_nosmooth":    "A* (nosmooth)",
}

# -- Cargar CSVs ---------------------------------------------------------------------------
def load_data():
    files = {
        "dijkstra":          "metrics_dijkstra_dinamico.csv",
        "dijkstra_nosmooth": "metrics_dijkstra_nosmooth_dinamico.csv",
        "astar":             "metrics_astar_dinamico.csv",
        "astar_nosmooth":    "metrics_astar_nosmooth_dinamico.csv",
    }
    data = {}
    for key, filename in files.items():
        path = os.path.join(RESULTS_DIR, filename)
        data[key] = pd.read_csv(path)
    return data


# -- Gráficas 1-4: Trayectoria real vs planificada ---------------------------------------------------------------------------
def plot_trajectories(data):
    for i, (key, df) in enumerate(data.items(), start=1):
        fig, ax = plt.subplots(figsize=(8, 8))

        # Trayectoria planificada
        ax.plot(df["pos_plan_x"], df["pos_plan_y"],
                color="gray", linewidth=1.5, linestyle="--",
                label="Trayectoria planificada", alpha=0.7)

        # Trayectoria real
        ax.plot(df["pos_real_x"], df["pos_real_y"],
                color=COLORS[key], linewidth=2,
                label="Trayectoria real")

        # Punto de inicio y fin
        ax.scatter(df["pos_real_x"].iloc[0],  df["pos_real_y"].iloc[0],
                   color="green", s=100, zorder=5, label="Inicio")
        ax.scatter(df["pos_real_x"].iloc[-1], df["pos_real_y"].iloc[-1],
                   color="red",   s=100, zorder=5, label="Fin")

        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_title(f"Trayectoria — {LABELS[key]}")
        ax.legend()
        ax.grid(True, linestyle="--", alpha=0.5)
        ax.set_aspect("equal")
        plt.tight_layout()
        path = os.path.join(PLOTS_DIR, f"{i}_trajectory_{key}.png")
        plt.savefig(path, dpi=150)
        plt.close()
        print(f"Guardada: {path}")


# -- Gráficas 5-8: Velocidades ---------------------------------------------------------------------------
def plot_velocities(data):
    for i, (key, df) in enumerate(data.items(), start=5):
        fig, ax1 = plt.subplots(figsize=(11, 5))

        # Eje izquierdo — velocidad lineal
        color_lineal = "#00CC00"   # verde
        ax1.plot(df["timestamp_s"], df["vel_lineal"],
                 color=color_lineal, linewidth=1.5,
                 label="Velocidad lineal (m/s)")
        ax1.set_xlabel("Tiempo (s)")
        ax1.set_ylabel("Velocidad lineal (m/s)", color=color_lineal)
        ax1.tick_params(axis="y", labelcolor=color_lineal)
        ax1.axhline(0, color="black", linewidth=0.8, linestyle=":")

        # Eje derecho — velocidad angular
        color_angular = "#FFA040"   # naranja
        ax2 = ax1.twinx()
        ax2.plot(df["timestamp_s"], df["vel_angular"],
                 color=color_angular, linewidth=1.5, linestyle="--",
                 label="Velocidad angular (rad/s)")
        ax2.set_ylabel("Velocidad angular (rad/s)", color=color_angular)
        ax2.tick_params(axis="y", labelcolor=color_angular)

        # Leyenda combinada de ambos ejes
        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(lines1 + lines2, labels1 + labels2)

        ax1.set_title(f"Velocidades — {LABELS[key]}")
        ax1.grid(True, linestyle="--", alpha=0.5)
        plt.tight_layout()
        path = os.path.join(PLOTS_DIR, f"{i}_velocities_{key}.png")
        plt.savefig(path, dpi=150)
        plt.close()
        print(f"Guardada: {path}")


# -- Gráficas 9-12: ETA real vs ETA calculado ---------------------------------------------------------------------------
def plot_eta_comparison(data):
    for i, (key, df) in enumerate(data.items(), start=9):
        t_final = df["timestamp_s"].iloc[-1]

        # ETA real restante = tiempo total - tiempo transcurrido
        eta_real = t_final - df["timestamp_s"]

        fig, ax = plt.subplots(figsize=(11, 5))

        ax.plot(df["timestamp_s"], eta_real,
                color="black", linewidth=2,
                label="Tiempo restante real (s)")
        ax.plot(df["timestamp_s"], df["eta_s"],
                color=COLORS[key], linewidth=1.5, linestyle="--",
                label="ETA calculado por Nav2 (s)")

        ax.set_xlabel("Tiempo transcurrido (s)")
        ax.set_ylabel("Tiempo restante (s)")
        ax.set_title(f"ETA real vs calculado — {LABELS[key]}")
        ax.legend()
        ax.grid(True, linestyle="--", alpha=0.5)
        plt.tight_layout()
        path = os.path.join(PLOTS_DIR, f"{i}_eta_{key}.png")
        plt.savefig(path, dpi=150)
        plt.close()
        print(f"Guardada: {path}")


# -- Gráfica 13: Tiempo total de navegación ---------------------------------------------------------------------------
def plot_total_time(data):
    keys   = list(data.keys())
    times  = [data[k]["timestamp_s"].iloc[-1] for k in keys]
    labels = [LABELS[k] for k in keys]
    colors = [COLORS[k] for k in keys]

    fig, ax = plt.subplots(figsize=(9, 5))
    x = np.arange(len(keys))
    bars = ax.bar(x, times, color=colors, width=0.5)

    # Etiqueta con el valor encima de cada barra
    for bar, t in zip(bars, times):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1,
                f"{t:.1f}s", ha="center", va="bottom", fontsize=10)

    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=15, ha="right")
    ax.set_ylabel("Tiempo total de navegación (s)")
    ax.set_title("Tiempo total de navegación — Comparativa")
    ax.grid(True, axis="y", linestyle="--", alpha=0.5)
    plt.tight_layout()
    path = os.path.join(PLOTS_DIR, "13_total_time.png")
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"Guardada: {path}")

# -- Gráfica 14: Comparativa de ETA calculado por cada algoritmo ---------------------------------------------------------------------------
def plot_eta_all(data):
    fig, ax = plt.subplots(figsize=(11, 5))

    for key, df in data.items():
        ax.plot(df["timestamp_s"], df["eta_s"],
                color=COLORS[key], linewidth=1.5,
                label=LABELS[key])

    ax.set_xlabel("Tiempo transcurrido (s)")
    ax.set_ylabel("ETA (s)")
    ax.set_title("ETA calculado por Nav2 — Comparativa")
    ax.legend()
    ax.grid(True, linestyle="--", alpha=0.5)
    plt.tight_layout()
    path = os.path.join(PLOTS_DIR, "14_eta_all.png")
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"Guardada: {path}")


# -- Main ---------------------------------------------------------------------------
if __name__ == "__main__":
    print("Cargando datos...")
    data = load_data()

    print("Generando gráficas...")
    plot_trajectories(data)
    plot_velocities(data)
    plot_eta_comparison(data)
    plot_total_time(data)
    plot_eta_all(data)

    print(f"\nTodas las gráficas guardadas en: {PLOTS_DIR}")