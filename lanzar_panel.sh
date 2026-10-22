#!/usr/bin/env bash
# Abre el panel del rover (doble clic o ./lanzar_panel.sh)
DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"

# Quita variables del VS Code snap que rompen las apps gráficas
unset GTK_PATH GTK_EXE_PREFIX GDK_PIXBUF_MODULE_FILE GDK_PIXBUF_MODULEDIR \
      GIO_MODULE_DIR GSETTINGS_SCHEMA_DIR LOCPATH GTK_IM_MODULE_FILE
export QT_QPA_PLATFORM=xcb

source /opt/ros/jazzy/setup.bash
[ -f "$DIR/install/setup.bash" ] && source "$DIR/install/setup.bash"

# Corre el archivo fuente: así siempre es la última versión, sin recompilar
exec python3 "$DIR/src/vision/vision/rover_panel.py"
