#!/bin/bash

# Lista de extensiones a instalar
extensions=(
    "oderwat.indent-rainbow"
    "ms-python.python"
    "PKief.material-icon-theme"
    "usernamehw.errorlens"
    "christian-kohler.path-intellisense"
    "ms-azuretools.vscode-docker"
    "donjayamanne.python-extension-pack"
    "ms-iot.vscode-ros"
)

# Ruta al ejecutable de Visual Studio Code
code_executable="code"

# Comando para instalar cada extensión
for extension in "${extensions[@]}"; do
    $code_executable --install-extension $extension
done

echo "Extensiones instaladas correctamente."

#chmod +x install_extensions.sh
#./install_extensions.sh
# Se tiene que lanzar el scrit desde VsCode conectado al contenedor