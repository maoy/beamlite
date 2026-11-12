#!/bin/bash
# Build BeamLite natively on macOS (Apple Silicon or Intel).
# Needs: Xcode Command Line Tools (xcode-select --install).
# Uses Homebrew raylib if installed, otherwise builds raylib 5.5 from source once.
set -e
cd "$(dirname "$0")"

if pkg-config --exists raylib 2>/dev/null; then
  CFLAGS="$(pkg-config --cflags raylib)"
  LIBS="$(pkg-config --libs raylib)"
else
  RL=third_party/raylib
  if [ ! -f "$RL/src/libraylib.a" ]; then
    echo ">> raylib not found via Homebrew — building raylib 5.5 from source (one time, ~1 min)"
    mkdir -p third_party
    [ -d "$RL" ] || git clone --depth 1 --branch 5.5 https://github.com/raysan5/raylib.git "$RL"
    make -C "$RL/src" PLATFORM=PLATFORM_DESKTOP -j"$(sysctl -n hw.ncpu)"
  fi
  CFLAGS="-I$RL/src"
  LIBS="$RL/src/libraylib.a -framework Cocoa -framework IOKit -framework CoreVideo -framework OpenGL"
fi

cc -O3 -std=c11 -Wall -DGL_SILENCE_DEPRECATION src/main.c src/physics.c $CFLAGS $LIBS -lm -o beamlite
echo ">> Built ./beamlite  — run:  ./beamlite"
