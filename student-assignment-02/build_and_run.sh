#!/bin/bash
# Build and Run Script za A* Path Planner
# Korištenje: ./build_and_run.sh [option]
# Opcije: build, run_all, clean

set -e  # Exit na prvoj grešci

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo "[INFO] Script dir: $SCRIPT_DIR"

# Boja za output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

# Funkcija: Clean build direktorija
clean_build() {
    print_info "Očistić stare build direktorije..."
    if [ -d "$SCRIPT_DIR/build" ]; then
        rm -rf "$SCRIPT_DIR/build"
        print_success "Obrisan build/ direktorij"
    fi
    if [ -d "$SCRIPT_DIR/install" ]; then
        rm -rf "$SCRIPT_DIR/install"
        print_success "Obrisan install/ direktorij"
    fi
    if [ -d "$SCRIPT_DIR/log" ]; then
        rm -rf "$SCRIPT_DIR/log"
        print_success "Obrisan log/ direktorij"
    fi
}

# Funkcija: Build paket
build_package() {
    print_info "Build-anje student_assignment_02 paketa..."
    cd "$SCRIPT_DIR"
    colcon build --packages-select student_assignment_02 --symlink-install
    print_success "Build završen!"
    
    # Source setup.bash
    print_info "Sourcing setup.bash..."
    source "$SCRIPT_DIR/install/setup.bash"
    print_success "Setup sourced!"
}

# Funkcija: Pokrenite sve čvorove
run_all() {
    print_info "Pokrećanje svih čvorova..."
    print_warning "Trebat će vam više terminala. Preporuka: koristiti tmux ili nove terminale"
    
    # Ovjeri da je setup sourced
    if ! command -v ros2 &> /dev/null; then
        print_error "ros2 nije dostupan! Mons ste source-ali setup.bash?"
        exit 1
    fi
    
    print_info ""
    print_info "========================================"
    print_info "Pokrenite ove naredbe u različitim terminalima:"
    print_info "========================================"
    print_info ""
    
    print_info "Terminal 1 - Stage Simulator:"
    echo "  ros2 launch stage_ros2 stage.launch.py"
    print_info ""
    
    print_info "Terminal 2 - Map Republisher:"
    echo "  ros2 run student_assignment_02 map_republisher"
    print_info ""
    
    print_info "Terminal 3 - A* Path Planner:"
    echo "  ros2 run student_assignment_02 a_star_path_planner"
    print_warning "(Ili koristite launch file: ros2 launch student_assignment_02 a_star_path_planner.launch.py)"
    print_info ""
    
    print_info "Terminal 4 - RViz:"
    echo "  ros2 run rviz2 rviz2"
    print_info ""
    
    print_info "========================================"
    print_success "Sve komande su sprema! Pokrenite ih u različitim terminalima."
}

# Funkcija: Prikazi pomoć
show_help() {
    echo "Build and Run Script za A* Path Planner"
    echo ""
    echo "Korištenje: $0 [option]"
    echo ""
    echo "Opcije:"
    echo "  build       - Build paket (očist i prebildaj)"
    echo "  clean       - Očist build direktorije"
    echo "  run_all     - Prikaži naredbe za pokretanje svih čvorova"
    echo "  help        - Prikaži ovu pomoć"
    echo ""
    echo "Primjer:"
    echo "  $0 build"
    echo "  $0 run_all"
}

# Main
case "${1:-help}" in
    build)
        clean_build
        build_package
        print_success "Build gotov! Sada možete pokrenuti: $0 run_all"
        ;;
    clean)
        clean_build
        print_success "Očistećenje gotovo!"
        ;;
    run_all)
        run_all
        ;;
    help)
        show_help
        ;;
    *)
        print_error "Nepoznata opcija: $1"
        show_help
        exit 1
        ;;
esac
