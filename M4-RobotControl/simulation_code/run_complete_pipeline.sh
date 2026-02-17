#!/bin/bash
# Automated pipeline for PID experiments on SO-101 robot
# Runs config generation, simulations, plots and metric analysis
# Must be run inside the Docker container

set -e

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo ""
echo "=========================================================================="
echo -e "${BLUE}PID Experiments Pipeline - SO-101${NC}"
echo "=========================================================================="
echo ""
echo "This will execute:"
echo "  1. Generation of 20 configs"
echo "  2. Simulation of 20 experiments (~10 min)"
echo "  3. Plots"
echo "  4. Metrics analysis"
echo ""
echo "=========================================================================="
echo ""

read -p "Continue? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo -e "${RED}Cancelled${NC}"
    exit 1
fi

echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

CONFIG_FILE="configs_rubric.json"
LOGS_DIR="logs_rubrica"
PLOTS_DIR="plots_rubrica"
METRICS_FILE="METRICS.md"
METRICS_JSON="metrics.json"

START_TIME=$(date +%s)

# Step 1: Clean previous directories
echo "=========================================================================="
echo -e "${YELLOW}[1/5] Cleaning previous files...${NC}"
echo "=========================================================================="

if [ -d "$LOGS_DIR" ]; then
    echo "Deleting $LOGS_DIR/"
    rm -rf "$LOGS_DIR"
fi

if [ -d "$PLOTS_DIR" ]; then
    echo "Deleting $PLOTS_DIR/"
    rm -rf "$PLOTS_DIR"
fi

if [ -f "$METRICS_FILE" ]; then
    echo "Deleting $METRICS_FILE"
    rm -f "$METRICS_FILE"
fi

if [ -f "$METRICS_JSON" ]; then
    echo "Deleting $METRICS_JSON"
    rm -f "$METRICS_JSON"
fi

echo -e "${GREEN}Cleanup completed${NC}"
echo ""

# Step 2: Generate configs
echo "=========================================================================="
echo -e "${YELLOW}[2/5] Generating PID configs...${NC}"
echo "=========================================================================="

python3 generate_20_configs.py --output "$CONFIG_FILE" --print-table

if [ ! -f "$CONFIG_FILE" ]; then
    echo -e "${RED}Error: failed to generate $CONFIG_FILE${NC}"
    exit 1
fi

echo -e "${GREEN}OK: $CONFIG_FILE created${NC}"
echo ""

# Step 3: Run experiments
echo "=========================================================================="
echo -e "${YELLOW}[3/5] Running 20 experiments (~10 min)...${NC}"
echo "=========================================================================="

EXP_START=$(date +%s)

python3 run_all_rubric_experiments.py \
    --config "$CONFIG_FILE" \
    --output "$LOGS_DIR/"

EXP_END=$(date +%s)
EXP_DURATION=$((EXP_END - EXP_START))

echo ""
echo -e "${GREEN}Experiments completed in ${EXP_DURATION}s${NC}"
echo ""

CSV_COUNT=$(ls -1 "$LOGS_DIR"/log_*.csv 2>/dev/null | wc -l)
echo "CSVs generated: $CSV_COUNT"
echo ""

# Step 4: Generate plots
echo "=========================================================================="
echo -e "${YELLOW}[4/5] Generating plots...${NC}"
echo "=========================================================================="

python3 plot_rubric_results.py \
    --input "$LOGS_DIR/" \
    --output "$PLOTS_DIR/"

PLOT_COUNT=$(ls -1 "$PLOTS_DIR"/plot_*.png 2>/dev/null | wc -l)
echo ""
echo -e "${GREEN}Plots created: $PLOT_COUNT${NC}"
echo ""

# Step 5: Calculate metrics
echo "=========================================================================="
echo -e "${YELLOW}[5/5] Calculating metrics...${NC}"
echo "=========================================================================="

python3 analyze_metrics.py \
    --input "$LOGS_DIR/" \
    --output "$METRICS_FILE" \
    --json "$METRICS_JSON"

echo ""
echo -e "${GREEN}Metrics calculated${NC}"
echo "   - Markdown: $METRICS_FILE"
echo "   - JSON: $METRICS_JSON"
echo ""

# Final summary
END_TIME=$(date +%s)
TOTAL_DURATION=$((END_TIME - START_TIME))

echo ""
echo "================================================================================"
echo -e "${GREEN}PIPELINE COMPLETED SUCCESSFULLY${NC}"
echo "================================================================================"
echo ""
echo "Total time: ${TOTAL_DURATION}s (~$((TOTAL_DURATION / 60)) minutes)"
echo ""
echo "Files generated:"
echo "  $LOGS_DIR/ → $CSV_COUNT CSVs"
echo "  $PLOTS_DIR/ → $PLOT_COUNT PNGs"
echo "  $METRICS_FILE → Metrics table"
echo "  $METRICS_JSON → JSON data"
echo ""
echo "=========================================================================="
echo ""
echo "To review results:"
echo "  cat $METRICS_FILE"
echo "  ls -lh $PLOTS_DIR/"
echo ""
echo "=========================================================================="
echo -e "${BLUE}Complete pipeline - T3001B${NC}"
echo "=========================================================================="
echo ""

exit 0
