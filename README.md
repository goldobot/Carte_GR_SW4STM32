
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

