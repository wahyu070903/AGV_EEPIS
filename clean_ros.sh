#!/bin/bash

if [ ! -d "src" ]; then
    echo "❌ Error: Folder 'src' tidak ditemukan! Pastikan Anda berada di root workspace ROS 2."
    exit 1
fi

echo "🧹 Membersihkan workspace ROS 2..."

TARGETS=("build" "install" "log")

for target in "${TARGETS[@]}"; do
    if [ -d "$target" ]; then
        rm -rf "$target"
        echo "   [HAPUS] Folder '$target/' berhasil dibersihkan."
    else
        echo "   [INFO] Folder '$target/' tidak ada (lewati)."
    fi
done

echo "✨ Workspace bersih! Siap untuk colcon build ulang."