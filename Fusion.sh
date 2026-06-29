#!/bin/bash
set -euo pipefail

GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

PACKAGE="localizacao_husky"
PACKAGE_DIR=$(rospack find "$PACKAGE") 

BAG="${1:-$PACKAGE_DIR/bags/sensores_husky.bag}"
RESULTS_DIR="${2:-$PACKAGE_DIR/results}"
MODES=(odom odom_imu odom_imu_gps)

if [ ! -f "$BAG" ]; then
    echo -e "${RED}Erro: Bag não encontrada em: $BAG${NC}" >&2
    exit 1
fi

mkdir -p "$RESULTS_DIR"

# --- Limpeza do arquivo unificado anterior ---
CSV_UNIFICADO="$RESULTS_DIR/todos_os_modos_metrics.csv"
rm -f "$CSV_UNIFICADO"

for MODE in "${MODES[@]}"; do
    echo -e "\n${GREEN}====* Configuração: $MODE *====${NC}"

    # Guarda o tamanho do arquivo antes de rodar o modo atual
    TAMANHO_ANTERIOR=0
    if [ -f "$CSV_UNIFICADO" ]; then
        TAMANHO_ANTERIOR=$(stat -c%s "$CSV_UNIFICADO")
    fi

    roslaunch "$PACKAGE" main.launch \
        mode:="$MODE" \
        bag:="$BAG" \
        output_dir:="$RESULTS_DIR"

    # --- Nova Validação de Segurança de Execução ---
    if [ ! -f "$CSV_UNIFICADO" ]; then
        echo -e "${RED}Erro crítico: O arquivo unificado não foi criado no modo $MODE!${NC}" >&2
        exit 1
    fi

    TAMANHO_ATUAL=$(stat -c%s "$CSV_UNIFICADO")
    if [ "$TAMANHO_ATUAL" -le "$TAMANHO_ANTERIOR" ]; then
        echo -e "${RED}Erro: Nenhuma métrica nova foi adicionada para o modo $MODE (o filtro pode ter falhado)!${NC}" >&2
        exit 1
    fi
done

# --- Finalização e Comparação ---
echo -e "\n${GREEN}====* Comparação *====${NC}"
rosrun "$PACKAGE" resultados.py --results-dir "$RESULTS_DIR"