# Installation System Workbench pour STM32
Allez sur le site [System Workbench pour STM32](https://www.openstm32.org/System+Workbench+for+STM32).
Créez un compte et télécharger le programme d'installation pour votre OS.
Installez System Workbench pour STM32.

# Import du projet
Ouvrez System Worbench pour STM32.
Cliquez sur Open Project from File System dans le menu File.
Cliquez sur le bouton Directory et sélectionnez le chemin du repository Carte_GR_SW4STM32.
![](images/build_system_workbench_1.png)
![](images/build_system_workbench_2.png)Cliquez sur Finish.

# Build
Sélectionnez la configuration de build.
Cliquez sur Build.
![](images/build_system_workbench_3.png)

# Setup Debug
Cliquez sur Debug Configurations dans le menu Run.
Faites un click droit sur GDB Hardware Debugging et sélectionnez l'option New.
Dans l'onglet Main, cliquez sur Browse et et sélectionnez le projet.
Dans l'onglet Debugger, cliquez sur Browse et sélectionnez le chemin du programme arm-none-eabi-gdb.exe.
Entrez l’adresse et le port utilisés par le débuggeur openocd. Vous pouvez utiliser localhost et vous connecter à la raspberry pi du robot en utilisant ssh avec l'option de forward de port.







# Carte_GR_SW4STM32
BSP pour la carte "Gros Robot" (Nucleo F303RE + DE0-Nano)

# Dépendances
La compilation nécessite CMake3.10 minimum et arm-none-eabi-gcc 5.
Ubuntu 18.04 contient des versions suffisamment récentes.

# Instructions pour linux
Installer les packages cmake et arm-none-eabi-gcc avec apt
Se placer à la racine du dépot et taper:
```shell
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchain/gcc-stm32f303xe.toolchain ..
cmake --build .
```
Le résultat du build est un fichier firmware.elf qui peut être utilisé pour débug avec gcc, et un fichier firmware.hex qui peut être copié sur la nucléo.

# Instructions pour windows
Installer CMake, arm-none-eabi-gcc et Ninja, et s'assurer qu'ils soient sur le path.
Se placer à la racine du dépot et taper:
```shell
mkdir build
cd build
cmake -G Ninja -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchain/gcc-stm32f303xe.toolchain ..
cmake --build .
```

