#!/usr/bin/env bash
#/|/ Copyright (c) preFlight 2025+ oozeBot, LLC
#/|/
#/|/ Released under AGPLv3 or higher
#/|/
# Pack a completed Linux build into an AppImage.
# Run ./build.sh first.
#
# Usage: ./pack_appimage.sh [options]
#   -build-dir DIR   Path to build directory (default: ./build)
#   -output DIR      Output directory for the .AppImage (default: ./releases)
#   -clean           Remove AppDir before assembling

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"
OUTPUT_DIR="$SCRIPT_DIR/releases"
CLEAN=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        -build-dir) BUILD_DIR="$2"; shift ;;
        -output)    OUTPUT_DIR="$2"; shift ;;
        -clean)     CLEAN=1 ;;
        -h|-help|--help)
            sed -n '8,13p' "$0"
            exit 0
            ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
    shift
done

ARCH="$(uname -m)"
case "$ARCH" in
    x86_64) PKG_ARCH="amd64" ;;
    aarch64) PKG_ARCH="aarch64" ;;
    *) PKG_ARCH="$ARCH" ;;
esac

VERSION="$(grep 'set(SLIC3R_VERSION ' "$SCRIPT_DIR/version.inc" | sed 's/.*"\(.*\)".*/\1/')"
if [[ -z "$VERSION" ]]; then
    echo "ERROR: Could not read version from version.inc"
    exit 1
fi

BINARY="$BUILD_DIR/src/preflight"
if [[ ! -x "$BINARY" ]]; then
    echo "ERROR: $BINARY not found. Run ./build.sh first."
    exit 1
fi

APPDIR="$BUILD_DIR/AppDir"
APPIMAGETOOL="$BUILD_DIR/appimagetool-$ARCH.AppImage"
REQ_GLIBC="$(objdump -T "$BINARY" | grep -o 'GLIBC_[0-9.]*' | sed 's/GLIBC_//' | sort -V | tail -1)"

echo "**********************************************************************"
echo "** preFlight AppImage Packager"
echo "** Version:    $VERSION"
echo "** Arch:       $ARCH ($PKG_ARCH)"
echo "** glibc need: $REQ_GLIBC"
echo "** Build dir:  $BUILD_DIR"
echo "** Output:     $OUTPUT_DIR"
echo "**********************************************************************"

if [[ ! -x "$APPIMAGETOOL" ]]; then
    echo "** Downloading appimagetool ..."
    URL="https://github.com/AppImage/appimagetool/releases/download/continuous/appimagetool-${ARCH}.AppImage"
    curl -fL --retry 3 -o "$APPIMAGETOOL" "$URL"
    chmod +x "$APPIMAGETOOL"
fi

if [[ "$CLEAN" -eq 1 && -d "$APPDIR" ]]; then
    rm -rf "$APPDIR"
fi
rm -rf "$APPDIR"
mkdir -p "$APPDIR/usr/bin" "$APPDIR/usr/lib/compat" "$APPDIR/usr/lib/softgl/dri" \
    "$APPDIR/usr/share/applications" "$APPDIR/usr/share/icons/hicolor/scalable/apps"

echo "** Installing executable, resources, and Python runtime ..."
cp -a "$BINARY" "$APPDIR/usr/bin/preflight"
chmod +x "$APPDIR/usr/bin/preflight"
ln -sfn preflight "$APPDIR/usr/bin/preflight-gcodeviewer"
cp -a "$SCRIPT_DIR/resources" "$APPDIR/usr/resources"
if [[ -d "$BUILD_DIR/python" ]]; then
    mkdir -p "$APPDIR/usr/python"
    cp -a "$BUILD_DIR/python/bin" "$BUILD_DIR/python/lib" "$BUILD_DIR/python/share" "$APPDIR/usr/python/"
fi

cp -a "$SCRIPT_DIR/src/platform/unix/preFlight.desktop" "$APPDIR/preFlight.desktop"
cp -a "$SCRIPT_DIR/src/platform/unix/preFlight.desktop" "$APPDIR/usr/share/applications/"
cp -a "$SCRIPT_DIR/resources/icons/preFlight.svg" "$APPDIR/preFlight.svg"
cp -a "$SCRIPT_DIR/resources/icons/preFlight.svg" "$APPDIR/.DirIcon"
cp -a "$SCRIPT_DIR/resources/icons/preFlight.svg" "$APPDIR/usr/share/icons/hicolor/scalable/apps/"

classify_lib() {
    local base="$1"
    case "$base" in
        ld-linux*.so*|libc.so.6|libm.so.6|libmvec.so.1|libdl.so.2|libpthread.so.0|librt.so.1|libresolv.so.2|libutil.so.1|libanl.so.1|libBrokenLocale.so.1|libthread_db.so.1|libnss_*.so*)
            echo compat ;;
        libnvidia*.so*|libcuda.so*|libnvcuvid.so*|libwayland*.so*)
            echo skip ;;
        libGL.so*|libGLX*.so*|libGLdispatch.so*|libEGL*.so*|libOpenGL.so*|libdrm.so*|libgbm.so*|libgallium*.so*|libglapi.so*)
            echo softgl ;;
        *)
            echo app ;;
    esac
}

declare -A SCANNED
SCAN_LIST=()

enqueue() {
    local real
    real="$(readlink -f "$1")"
    [[ -n "$real" && -e "$real" ]] || return 0
    if [[ -n "${SCANNED[$real]:-}" ]]; then
        return 0
    fi
    SCANNED[$real]=1
    SCAN_LIST+=("$real")
}

install_lib() {
    local dep="$1"
    local class="$2"
    local dest real name target_name
    case "$class" in
        app) dest="$APPDIR/usr/lib" ;;
        compat) dest="$APPDIR/usr/lib/compat" ;;
        softgl) dest="$APPDIR/usr/lib/softgl" ;;
        *) return 0 ;;
    esac
    real="$(readlink -f "$dep")"
    [[ -f "$real" ]] || return 0
    name="$(basename "$dep")"
    target_name="$(basename "$real")"
    mkdir -p "$dest"
    if [[ ! -e "$dest/$target_name" ]]; then
        cp -a "$real" "$dest/$target_name"
    fi
    if [[ "$name" != "$target_name" && ! -e "$dest/$name" ]]; then
        ln -s "$target_name" "$dest/$name"
    fi
}

enqueue "$BINARY"

# WebKit subprocesses. Only the injected-bundle directory is compiled into
# libwebkit as an absolute path; the processes live next to that directory.
WEBKIT_SRC=""
for d in /usr/lib/webkit2gtk-4.1 /usr/lib64/webkit2gtk-4.1 /usr/lib/x86_64-linux-gnu/webkit2gtk-4.1; do
    if [[ -x "$d/WebKitWebProcess" ]]; then
        WEBKIT_SRC="$d"
        break
    fi
done
if [[ -n "$WEBKIT_SRC" ]]; then
    echo "** Bundling WebKit subprocesses from $WEBKIT_SRC"
    mkdir -p "$APPDIR/usr/lib/webkit2gtk-4.1/injected-bundle"
    for proc in WebKitWebProcess WebKitNetworkProcess WebKitGPUProcess; do
        if [[ -x "$WEBKIT_SRC/$proc" ]]; then
            cp -a "$WEBKIT_SRC/$proc" "$APPDIR/usr/lib/webkit2gtk-4.1/$proc"
            chmod +x "$APPDIR/usr/lib/webkit2gtk-4.1/$proc"
            enqueue "$WEBKIT_SRC/$proc"
        fi
    done
    if [[ -f "$WEBKIT_SRC/injected-bundle/libwebkit2gtkinjectedbundle.so" ]]; then
        cp -a "$WEBKIT_SRC/injected-bundle/libwebkit2gtkinjectedbundle.so" \
            "$APPDIR/usr/lib/webkit2gtk-4.1/injected-bundle/"
        enqueue "$WEBKIT_SRC/injected-bundle/libwebkit2gtkinjectedbundle.so"
    fi
else
    echo "** WARNING: WebKit subprocesses not found; the embedded browser may not work"
fi

if [[ -d /usr/lib/gtk-3.0 ]]; then
    cp -a /usr/lib/gtk-3.0 "$APPDIR/usr/lib/"
    while IFS= read -r -d '' so; do
        enqueue "$so"
    done < <(find "$APPDIR/usr/lib/gtk-3.0" -type f -name '*.so' -print0)
fi
if [[ -d /usr/lib/gio/modules ]]; then
    mkdir -p "$APPDIR/usr/lib/gio/modules"
    cp -a /usr/lib/gio/modules/. "$APPDIR/usr/lib/gio/modules/"
    while IFS= read -r -d '' so; do
        enqueue "$so"
    done < <(find /usr/lib/gio/modules -type f -name '*.so' -print0)
fi
if [[ -d "$APPDIR/usr/python/lib" ]]; then
    while IFS= read -r -d '' so; do
        enqueue "$so"
    done < <(find "$APPDIR/usr/python/lib" -type f -name '*.so' -print0)
fi

echo "** Collecting shared libraries ..."
idx=0
while [[ "$idx" -lt "${#SCAN_LIST[@]}" ]]; do
    elf="${SCAN_LIST[$idx]}"
    idx=$((idx + 1))
    while IFS= read -r line; do
        dep=""
        if [[ "$line" == *"=>"* ]]; then
            dep="${line#*=> }"
            dep="${dep%% *}"
            [[ "$dep" == "not" || -z "$dep" ]] && continue
        elif [[ "$line" == *ld-linux* ]]; then
            dep="${line%% *}"
            dep="${dep#"${dep%%[![:space:]]*}"}"
        else
            continue
        fi
        [[ -e "$dep" ]] || continue
        class="$(classify_lib "$(basename "$dep")")"
        [[ "$class" == "skip" ]] && continue
        install_lib "$dep" "$class"
        enqueue "$dep"
    done < <(ldd "$elf" 2>/dev/null || true)
done

echo "** Adding Mesa software-GL fallback ..."
for extra in /usr/lib/libGLX_mesa.so.0 /usr/lib/libEGL_mesa.so.0 /usr/lib64/libGLX_mesa.so.0 /usr/lib64/libEGL_mesa.so.0; do
    if [[ -e "$extra" ]]; then
        install_lib "$extra" softgl
        enqueue "$extra"
    fi
done
if [[ -d /usr/lib/dri ]]; then
    for drv in swrast_dri.so kms_swrast_dri.so libdril_dri.so; do
        if [[ -e "/usr/lib/dri/$drv" ]]; then
            real="$(readlink -f "/usr/lib/dri/$drv")"
            cp -a "$real" "$APPDIR/usr/lib/softgl/dri/$(basename "$real")"
            if [[ "$(basename "$real")" != "$drv" ]]; then
                ln -sfn "$(basename "$real")" "$APPDIR/usr/lib/softgl/dri/$drv"
            fi
            enqueue "$real"
        fi
    done
fi
# Walk newly queued Mesa libraries.
while [[ "$idx" -lt "${#SCAN_LIST[@]}" ]]; do
    elf="${SCAN_LIST[$idx]}"
    idx=$((idx + 1))
    while IFS= read -r line; do
        dep=""
        if [[ "$line" == *"=>"* ]]; then
            dep="${line#*=> }"
            dep="${dep%% *}"
            [[ "$dep" == "not" || -z "$dep" ]] && continue
        else
            continue
        fi
        [[ -e "$dep" ]] || continue
        class="$(classify_lib "$(basename "$dep")")"
        [[ "$class" == "skip" ]] && continue
        install_lib "$dep" "$class"
        enqueue "$dep"
    done < <(ldd "$elf" 2>/dev/null || true)
done

python3 - << PY
from pathlib import Path
root = Path("$APPDIR/usr/lib")
old = b"/usr/lib/webkit2gtk-4.1/injected-bundle/"
# Same length as the original: "/usr" (4) is replaced by "././" (4).
new = b"././" + old[len(b"/usr"):]
if len(old) != len(new):
    raise SystemExit(f"WebKit path patch length mismatch: {len(old)} vs {len(new)}")
patched = 0
for lib in root.glob("libwebkit2gtk*.so*"):
    if not lib.is_file() or lib.is_symlink():
        continue
    data = lib.read_bytes()
    count = data.count(old)
    if count:
        lib.write_bytes(data.replace(old, new))
        print(f"** Patched {count} WebKit path(s) in {lib.name}")
        patched += count
if patched == 0:
    print("** WARNING: WebKit injected-bundle path was not found to patch")
PY

# The dynamic linker must keep its basename so the kernel can use it.
if [[ ! -e "$APPDIR/usr/lib/compat/ld-linux-x86-64.so.2" ]]; then
    ldso="$(readlink -f /lib64/ld-linux-x86-64.so.2)"
    cp -a "$ldso" "$APPDIR/usr/lib/compat/ld-linux-x86-64.so.2"
fi
chmod +x "$APPDIR/usr/lib/compat/ld-linux-x86-64.so.2"
# NSS modules are dlopened by the bundled libc. They are not NEEDED entries,
# so copy the ones name lookup needs when the host glibc is too old.
for nss in /usr/lib/libnss_files.so.2 /usr/lib/libnss_dns.so.2 /usr/lib/libnss_resolve.so.2 \
           /lib/libnss_files.so.2 /lib/libnss_dns.so.2 /lib/libnss_resolve.so.2; do
    if [[ -e "$nss" ]]; then
        install_lib "$nss" compat
    fi
done

cat > "$APPDIR/AppRun" << EOF
#!/bin/bash
# Three launch paths:
# 1. Host glibc is new enough: system linker and the host GPU driver.
# 2. Older glibc, no NVIDIA: bundled glibc, host GPU driver.
# 3. Older glibc with NVIDIA: bundled glibc and Mesa software GL.
#    Loading the NVIDIA driver into a process that uses a different glibc
#    crashes on GLIBC_PRIVATE.
HERE="\$(dirname "\$(readlink -f "\$0")")"
REQ_GLIBC="${REQ_GLIBC}"
cd "\$HERE/usr" || exit 1

if [[ -d "\$HERE/usr/lib/gtk-3.0" ]]; then
    export GTK_PATH="\$HERE/usr/lib/gtk-3.0\${GTK_PATH:+:\$GTK_PATH}"
fi
if [[ -d "\$HERE/usr/lib/gio/modules" ]]; then
    export GIO_MODULE_DIR="\$HERE/usr/lib/gio/modules"
fi
export XDG_DATA_DIRS="\$HERE/usr/share\${XDG_DATA_DIRS:+:\$XDG_DATA_DIRS}"

ver_ge() {
    [[ "\$(printf '%s\n%s\n' "\$1" "\$2" | sort -V | head -n1)" == "\$2" ]]
}

host_glibc="\$(getconf GNU_LIBC_VERSION 2>/dev/null | awk '{print \$2}')"
nvidia=0
if [[ -e /proc/driver/nvidia/version ]] || compgen -G '/usr/lib/libGLX_nvidia.so*' >/dev/null || compgen -G '/usr/lib64/libGLX_nvidia.so*' >/dev/null || compgen -G '/usr/lib/x86_64-linux-gnu/libGLX_nvidia.so*' >/dev/null; then
    nvidia=1
fi

SYS_LIB="/usr/lib/x86_64-linux-gnu:/usr/lib64:/usr/lib:/lib64:/lib:/usr/lib64"

if [[ -n "\$host_glibc" ]] && ver_ge "\$host_glibc" "\$REQ_GLIBC"; then
    export LD_LIBRARY_PATH="\$HERE/usr/lib\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH}"
    exec ./bin/preflight "\$@"
fi

unset LD_LIBRARY_PATH
if [[ "\$nvidia" -eq 0 ]]; then
    exec ./lib/compat/ld-linux-x86-64.so.2 \\
        --library-path "./lib:./lib/compat:\${SYS_LIB}" \\
        ./bin/preflight "\$@"
fi

export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
export __GLX_VENDOR_LIBRARY_NAME=mesa
export LIBGL_DRIVERS_PATH="\$HERE/usr/lib/softgl/dri"
exec ./lib/compat/ld-linux-x86-64.so.2 \\
    --library-path "./lib/softgl:./lib:./lib/compat" \\
    ./bin/preflight "\$@"
EOF
chmod +x "$APPDIR/AppRun"

echo "** Library layout:"
echo "   app libs:    $(find "$APPDIR/usr/lib" -maxdepth 1 -type f | wc -l)"
echo "   compat libs: $(find "$APPDIR/usr/lib/compat" -maxdepth 1 -type f | wc -l)"
echo "   softgl libs: $(find "$APPDIR/usr/lib/softgl" -maxdepth 1 -type f | wc -l)"

mkdir -p "$OUTPUT_DIR"
APPIMAGE_FILE="$OUTPUT_DIR/preFlight-${VERSION}-linux-${PKG_ARCH}.AppImage"
rm -f "$APPIMAGE_FILE"

echo "** Creating AppImage ..."
export APPIMAGE_EXTRACT_AND_RUN=1
ARCH="$ARCH" "$APPIMAGETOOL" --comp zstd "$APPDIR" "$APPIMAGE_FILE"

chmod +x "$APPIMAGE_FILE"
SIZE="$(du -h "$APPIMAGE_FILE" | cut -f1)"
echo ""
echo "**********************************************************************"
echo "** AppImage created"
echo "** File: $APPIMAGE_FILE"
echo "** Size: $SIZE"
echo "**********************************************************************"
