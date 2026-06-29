#!/usr/bin/env python
import argparse
import csv
import math
import os

DEFAULT_MODES = ["odom", "odom_imu", "odom_imu_gps"]
METRICS = [
    ("samples", "Amostras"),
    ("rmse_position_m", "RMSE Pos (m)"),
    ("final_position_error_m", "Erro F. Pos (m)"),
    ("rmse_yaw_rad", "RMSE Yaw (rad)"),
    ("final_yaw_error_rad", "Erro F. Yaw (rad)"),
]


def read_unified_metrics(path, target_modes):
    metrics_by_mode = {mode: [] for mode in target_modes}
    summary_data = {}

    if not os.path.exists(path):
        return metrics_by_mode, summary_data

    with open(path) as metrics_file:
        reader = csv.DictReader(metrics_file)
        for row in reader:
            mode = row.get("mode")
            if mode not in metrics_by_mode:
                continue
            try:
                metrics_by_mode[mode].append({
                    "time": float(row["time"]),
                    "filtered_x": float(row["filtered_x"]),
                    "filtered_y": float(row["filtered_y"]),
                    "gt_x": float(row["gt_x"]),
                    "gt_y": float(row["gt_y"]),
                    "position_error": float(row["position_error"]),
                    "yaw_error_rad": float(row["yaw_error_rad"]),
                })
            except (KeyError, TypeError, ValueError):
                continue

    for mode, rows in metrics_by_mode.items():
        if not rows:
            continue
        
        count = len(rows)
        sum_sq_pos = sum(r["position_error"] ** 2 for r in rows)
        sum_sq_yaw = sum(r["yaw_error_rad"] ** 2 for r in rows)
        
        rmse_pos = math.sqrt(sum_sq_pos / count)
        rmse_yaw = math.sqrt(sum_sq_yaw / count)
        final_pos_err = rows[-1]["position_error"]
        final_yaw_err = rows[-1]["yaw_error_rad"]

        summary_data[mode] = {
            "samples": str(int(count)),
            "rmse_position_m": "%.5f" % rmse_pos,
            "final_position_error_m": "%.5f" % final_pos_err,
            "rmse_yaw_rad": "%.5f" % rmse_yaw,
            "final_yaw_error_rad": "%.5f" % final_yaw_err,
        }

    return metrics_by_mode, summary_data


def generate_table_rows(summary_data, target_modes):
    rows = []
    for mode in target_modes:
        if mode in summary_data:
            data = summary_data[mode]
            mode_label = mode.upper().replace("_", " + ")
            rows.append([mode_label] + [data.get(key, "-") for key, _ in METRICS])
    return rows


def build_table_string(rows):
    headers = ["Configuração"] + [label for _, label in METRICS]
    
    widths = [len(header) for header in headers]
    for row in rows:
        widths = [max(width, len(str(value))) for width, value in zip(widths, row)]

    lines = []

    def format_row(values):
        formatted = []
        for i, val in enumerate(values):
            if i == 0:
                formatted.append(str(val).ljust(widths[i]))
            else:
                formatted.append(str(val).rjust(widths[i]))
        return "| " + " | ".join(formatted) + " |"

    lines.append(format_row(headers))
    
    div_parts = []
    for i, width in enumerate(widths):
        if i == 0:
            div_parts.append("-" * width)  
        else:
            div_parts.append("-" * (width - 1) + ":")  
    lines.append("| " + " | ".join(div_parts) + " |")
    
    for row in rows:
        lines.append(format_row(row))
        
    return "\n".join(lines)


def plot_results(results_dir, metrics_by_mode):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib nao esta disponivel, nao foram gerados plots de comparacao")
        return []

    if not any(metrics_by_mode.values()):
        return []

    paths = []

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    for mode, rows in metrics_by_mode.items():
        if not rows:
            continue
        t0 = rows[0]["time"]
        times = [row["time"] - t0 for row in rows]
        
        # Erro de Posição
        errors = [row["position_error"] for row in rows]
        axes[0].plot(times, errors, label=mode)
        
        # Erro de Orientação (Yaw)
        yaw_errors = [row["yaw_error_rad"] for row in rows]
        axes[1].plot(times, yaw_errors, label=mode)
    
    axes[0].set_title("Erro de posicao de cada modo")
    axes[0].set_xlabel("tempo [segundos]")
    axes[0].set_ylabel("erro [metros]")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].set_title("Erro de yaw de cada modo")
    axes[1].set_xlabel("tempo [segundos]")
    axes[1].set_ylabel("erro [radianos]")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    fig.tight_layout()
    error_path = os.path.join(results_dir, "Comparacao_de_Erros.png")
    fig.savefig(error_path, dpi=140)
    plt.close(fig)
    paths.append(error_path)


    fig, ax = plt.subplots(figsize=(7, 7))
    
    for mode, rows in metrics_by_mode.items():
        if rows:
            ax.plot([row["gt_x"] for row in rows],
                    [row["gt_y"] for row in rows],
                    color="red", linewidth=2.0, linestyle="--", label="ground Truth")
            break

    for mode, rows in metrics_by_mode.items():
        if not rows:
            continue
        ax.plot([row["filtered_x"] for row in rows],
                [row["filtered_y"] for row in rows],
                label=mode, alpha=0.8)
                
    ax.set_title("Trajetorias por modo vs ground truth")
    ax.set_xlabel("x [metros]")
    ax.set_ylabel("y [metros]")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    trajectory_path = os.path.join(results_dir, "Trajetorias.png")
    fig.savefig(trajectory_path, dpi=140)
    plt.close(fig)
    paths.append(trajectory_path)

    return paths


def main():
    parser = argparse.ArgumentParser(description="Comparando dados de localização a partir de um arquivo de métricas unificado.")
    parser.add_argument("--results-dir", default=os.path.expanduser("~/catkin_ws/src/localizacao_husky/results"))
    parser.add_argument("--modes", nargs="+", default=DEFAULT_MODES)
    parser.add_argument("--no-plots", action="store_true", help="Imprimindo e salvando a tabela de resumo.")
    args = parser.parse_args()

    unified_csv_path = os.path.join(args.results_dir, "todos_os_modos_metrics.csv")
    
    if not os.path.exists(unified_csv_path):
        print("Arquivo de metricas unificado nao encontrado: %s" % unified_csv_path)
        return 1

    metrics_by_mode, summary_data = read_unified_metrics(unified_csv_path, args.modes)

    table_rows = generate_table_rows(summary_data, args.modes)
    if not table_rows:
        print("Nenhum dado valido pôde ser extraído do arquivo unificado.")
        return 1

    table_output = build_table_string(table_rows)
    
    print("\n" + table_output)

    summary_txt_path = os.path.join(args.results_dir, "todos_os_modos_summary.txt")
    with open(summary_txt_path, "w") as f:
        f.write("=======================================================\n")
        f.write("          RESUMO COMPARATIVO DE LOCALIZACAO\n")
        f.write("=======================================================\n\n")
        f.write(table_output + "\n")
    print("\n[INFO] Resumo consolidado salvo em: %s" % summary_txt_path)

    if not args.no_plots:
        plot_paths = plot_results(args.results_dir, metrics_by_mode)
        if plot_paths:
            print("\nPlots gerados:")
            for path in plot_paths:
                print("= %s" % path)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())