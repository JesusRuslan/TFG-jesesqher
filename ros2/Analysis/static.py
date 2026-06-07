"""
static.py
Análisis de métricas estáticas de planificación de trayectorias.
Compara Dijkstra vs A* con y sin suavizado.

Gráficas generadas:
  1. Waypoints vs Tiempo de planificación (scatter, 4 algoritmos)
  2. Irregularidad: Dijkstra-smooth vs Dijkstra-nosmooth (barras por goal)
  3. Irregularidad: A*-smooth vs A*-nosmooth (barras por goal)
  4. Irregularidad — Comparativa completa (4 variantes, barras por goal)
  5. Curvatura media: Dijkstra-smooth vs A*-smooth (barras por goal)
  6. Curvatura media: Dijkstra-nosmooth vs A*-nosmooth (barras por goal)
  7. Curvatura media — Comparativa completa (4 variantes, barras por goal)
"""

import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# -- Configuración ---------------------------------------------------------------------------
RESULTS_DIR = os.path.join(os.path.dirname(__file__), "results", "static")
PLOTS_DIR   = os.path.join(os.path.dirname(__file__), "plots", "static")
os.makedirs(PLOTS_DIR, exist_ok=True)

# -- Colores por algoritmo ---------------------------------------------------------------------------
COLORS = {
    "dijkstra":          "#0000FF",   # azul
    "dijkstra_nosmooth": "#8888FF",   # azul claro
    "astar":             "#FF0000",   # rojo
    "astar_nosmooth":    "#FF8888",   # rojo claro
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
        "dijkstra":          "metrics_dijkstra.csv",
        "dijkstra_nosmooth": "metrics_dijkstra_nosmooth.csv",
        "astar":             "metrics_astar.csv",
        "astar_nosmooth":    "metrics_astar_nosmooth.csv",
    }
    data = {}
    for key, filename in files.items():
        path = os.path.join(RESULTS_DIR, filename)
        df = pd.read_csv(path)
        # Nombres de goals más cortos para las gráficas
        df["goal_label"] = df["goal_name"].str.replace("goal", "G", regex=False)
        data[key] = df
    return data


# -- Gráfica 1: Waypoints vs Tiempo de planificación ---------------------------------------------------------------------------
def plot_waypoints_vs_plantime(data):
    fig, ax = plt.subplots(figsize=(9, 5))

    for key, df in data.items():
        ax.scatter(
            df["num_waypoints"],
            df["plan_time_ms"],
            color=COLORS[key],
            label=LABELS[key],
            s=60,
            zorder=3,
        )
        # Etiqueta de cada punto con el nombre del goal
        for _, row in df.iterrows():
            ax.annotate(
                row["goal_label"],
                (row["num_waypoints"], row["plan_time_ms"]),
                textcoords="offset points",
                xytext=(5, 4),
                fontsize=7,
                color=COLORS[key],
            )

    ax.set_xlabel("Número de waypoints")
    ax.set_ylabel("Tiempo de planificación (ms)")
    ax.set_title("Waypoints vs Tiempo de planificación")
    ax.legend()
    ax.grid(True, linestyle="--", alpha=0.5)
    plt.tight_layout()
    path = os.path.join(PLOTS_DIR, "1_waypoints_vs_plantime.png")
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"Guardada: {path}")


# -- Gráficas 2 y 3: Irregularidad smooth vs nosmooth ---------------------------------------------------------------------------
def plot_smoothness_comparison(data):
    comparisons = [
        ("dijkstra", "dijkstra_nosmooth", "Irregularidad de la trayectoria — Dijkstra", "2_smoothness_dijkstra.png"),
        ("astar",    "astar_nosmooth",    "Irregularidad de la trayectoria — A*",       "3_smoothness_astar.png"),
    ]

    for key_a, key_b, title, filename in comparisons:
        df_a = data[key_a]
        df_b = data[key_b]

        goals  = df_a["goal_label"].tolist()
        x      = np.arange(len(goals))
        width  = 0.35

        fig, ax = plt.subplots(figsize=(9, 5))
        ax.bar(x - width/2, df_a["smoothness_rad"], width,
               color=COLORS[key_a], label=LABELS[key_a])
        ax.bar(x + width/2, df_b["smoothness_rad"], width,
               color=COLORS[key_b], label=LABELS[key_b])

        ax.set_xlabel("Goal")
        ax.set_ylabel("Cambio de ángulo acumulado (rad)")
        ax.set_title(title)
        ax.set_xticks(x)
        ax.set_xticklabels(goals)
        ax.legend()
        ax.grid(True, axis="y", linestyle="--", alpha=0.5)
        plt.tight_layout()
        path = os.path.join(PLOTS_DIR, filename)
        plt.savefig(path, dpi=150)
        plt.close()
        print(f"Guardada: {path}")

# -- Gráfica 4: Irregularidad total de cada variante ---------------------------------------------------------------------------
def plot_smoothness_all_comparison(data):
    goals   = data["dijkstra"]["goal_label"].tolist()
    x       = np.arange(len(goals))
    width   = 0.2

    fig, ax = plt.subplots(figsize=(11, 5))

    offsets = [-1.5, -0.5, 0.5, 1.5]
    keys    = ["dijkstra", "dijkstra_nosmooth", "astar", "astar_nosmooth"]

    for offset, key in zip(offsets, keys):
        ax.bar(x + offset * width,
               data[key]["smoothness_rad"],
               width,
               color=COLORS[key],
               label=LABELS[key])

    ax.set_xlabel("Goal")
    ax.set_ylabel("Cambio de ángulo acumulado (rad)")
    ax.set_title("Irregularidad de la trayectoria — Comparativa completa")
    ax.set_xticks(x)
    ax.set_xticklabels(goals)
    ax.legend()
    ax.grid(True, axis="y", linestyle="--", alpha=0.5)
    plt.tight_layout()
    path = os.path.join(PLOTS_DIR, "4_smoothness_all.png")
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"Guardada: {path}")

# -- Gráficas 5 y 6: Curvatura media Dijkstra vs A* ---------------------------------------------------------------------------
def plot_curvature_comparison(data):
    comparisons = [
        ("dijkstra",            "astar",            "Curvatura media — Smooth",     "5_curvature_smooth.png"),
        ("dijkstra_nosmooth",   "astar_nosmooth",   "Curvatura media — Nosmooth",   "6_curvature_nosmooth.png"),
    ]

    for key_a, key_b, title, filename in comparisons:
        df_a = data[key_a]
        df_b = data[key_b]

        goals  = df_a["goal_label"].tolist()
        x      = np.arange(len(goals))
        width  = 0.35

        fig, ax = plt.subplots(figsize=(9, 5))
        ax.bar(x - width/2, df_a["mean_curvature_rad_m"], width,
               color=COLORS[key_a], label=LABELS[key_a])
        ax.bar(x + width/2, df_b["mean_curvature_rad_m"], width,
               color=COLORS[key_b], label=LABELS[key_b])

        ax.set_xlabel("Goal")
        ax.set_ylabel("Curvatura media (rad/m)")
        ax.set_title(title)
        ax.set_xticks(x)
        ax.set_xticklabels(goals)
        ax.legend()
        ax.grid(True, axis="y", linestyle="--", alpha=0.5)
        plt.tight_layout()
        path = os.path.join(PLOTS_DIR, filename)
        plt.savefig(path, dpi=150)
        plt.close()
        print(f"Guardada: {path}")

# -- Gráfica 7: Curvatura media de cada variante ---------------------------------------------------------------------------
def plot_curvature_all_comparison(data):
    goals   = data["dijkstra"]["goal_label"].tolist()
    x       = np.arange(len(goals))
    width   = 0.2

    fig, ax = plt.subplots(figsize=(11, 5))

    offsets = [-1.5, -0.5, 0.5, 1.5]
    keys    = ["dijkstra", "dijkstra_nosmooth", "astar", "astar_nosmooth"]

    for offset, key in zip(offsets, keys):
        ax.bar(x + offset * width,
               data[key]["mean_curvature_rad_m"],
               width,
               color=COLORS[key],
               label=LABELS[key])

    ax.set_xlabel("Goal")
    ax.set_ylabel("Curvatura media (rad/m)")
    ax.set_title("Curvatura media — Comparativa completa")
    ax.set_xticks(x)
    ax.set_xticklabels(goals)
    ax.legend()
    ax.grid(True, axis="y", linestyle="--", alpha=0.5)
    plt.tight_layout()
    path = os.path.join(PLOTS_DIR, "7_curvature_all.png")
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"Guardada: {path}")


# -- Main ---------------------------------------------------------------------------
if __name__ == "__main__":
    print("Cargando datos...")
    data = load_data()

    print("Generando gráficas...")
    plot_waypoints_vs_plantime(data)
    plot_smoothness_comparison(data)
    plot_smoothness_all_comparison(data)
    plot_curvature_comparison(data)
    plot_curvature_all_comparison(data)

    print(f"\nTodas las gráficas guardadas en: {PLOTS_DIR}")