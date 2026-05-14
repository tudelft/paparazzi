#!/usr/bin/env bash
set -euo pipefail

# export DEPLOY_LOCALE=1
# export DEPLOY_OPENGL=1 
# export DEPLOY_VULKAN=1 
# export DEPLOY_PIPEWIRE=1
# export ANYLINUX_LIB=1

# Ensure robust out-of-tree execution defaults with absolute paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$(realpath -m "${1:-build_appimage}")"
REPO_ROOT="$(realpath -m "${2:-${SCRIPT_DIR}}")"
export ARCH="${ARCH:-x86_64}"

echo "--- Building EquinoxGCS AppImage for ${ARCH} ---"
echo "Repository Root: ${REPO_ROOT}"
echo "Build Directory: ${BUILD_DIR}"

# --- Dependency checks ---
for cmd in cmake wget make; do
    if ! command -v "$cmd" > /dev/null; then
        echo "Error: Required command '$cmd' is not installed or not in PATH." >&2
        exit 1
    fi
done

# --- Safe Directory preparation ---
# Prevent extremely dangerous automated wipes if variables are somehow screwed up
if [ "${BUILD_DIR}" = "/" ] || [ "${BUILD_DIR}" = "${HOME}" ]; then
    echo "Error: BUILD_DIR (${BUILD_DIR}) is too dangerous to clean automatically." >&2
    exit 1
fi

mkdir -p "${BUILD_DIR}"
# Clean existing contents properly
rm -rf "${BUILD_DIR:?}/"*

# Switch to build directory
pushd "${BUILD_DIR}" > /dev/null

# --- CMake Configuration ---
# Ensure Release build type for optimal distribution binaries
# Explicitly set the install prefix to /usr as mandated by linuxdeploy
echo "--- Configuring project ---"

# Clean up accidental in-source CMakeCache which breaks out-of-source builds
if [ -f "${REPO_ROOT}/CMakeCache.txt" ] && [ "${REPO_ROOT}" != "${BUILD_DIR}" ]; then
    echo "Warning: Found CMakeCache.txt in REPO_ROOT. Removing to allow clean out-of-source build."
    rm -f "${REPO_ROOT}/CMakeCache.txt"
fi

cmake -S "${REPO_ROOT}" -B . \
    -DCMAKE_INSTALL_PREFIX=/usr \
    -DCMAKE_BUILD_TYPE=Release

# --- Build Project ---
echo "--- Compiling and Installing into AppDir ---"
cmake --build . --parallel "$(nproc)"
make install DESTDIR=AppDir

# --- Download Linuxdeploy tooling ---
echo "--- Acquiring linuxdeploy tooling ---"
LINUXDEPLOY_URL="https://github.com/linuxdeploy/linuxdeploy/releases/download/continuous/linuxdeploy-${ARCH}.AppImage"
LINUXDEPLOY_QT_URL="https://github.com/linuxdeploy/linuxdeploy-plugin-qt/releases/download/continuous/linuxdeploy-plugin-qt-${ARCH}.AppImage"

# Download with continuation/resume, enforcing specific output filenames internally
wget -c "${LINUXDEPLOY_URL}" -O linuxdeploy.AppImage
wget -c "${LINUXDEPLOY_QT_URL}" -O linuxdeploy-plugin-qt.AppImage

# Make tools executable
chmod +x linuxdeploy*.AppImage

# --- Qt Toolkit Discovery ---
# Robust discovery of the qmake binary handling different Qt versions and distributions
if command -v qmake6 > /dev/null; then
    QMAKE_BIN="$(command -v qmake6)"
elif command -v qmake-qt5 > /dev/null; then
    QMAKE_BIN="$(command -v qmake-qt5)"
elif command -v qmake > /dev/null; then
    QMAKE_BIN="$(command -v qmake)"
else
    echo "Error: qmake/qmake6/qmake-qt5 not found. A valid Qt installation is required." >&2
    exit 1
fi

echo "Using Qt qmake from: ${QMAKE_BIN}"
# Setup qmake symbolic link in current path to be resolvable by linuxdeploy-plugin-qt
ln -sf "${QMAKE_BIN}" qmake
export PATH="$(pwd):${PATH}"

# --- Setup AppImage Metadata ---
# Extracting version dynamically from git if VERSION is not manually specified
if [ -z "${VERSION:-}" ]; then
    export VERSION="$(git -C "${REPO_ROOT}" describe --tags --always 2>/dev/null || echo "continuous")"
    echo "Setting AppImage VERSION to: ${VERSION}"
fi

# --- Executing Linuxdeploy ---
echo "--- Constructing AppImage ---"
# Initialize AppDir, bundle Qt shared libraries, inject extra resources, and generate AppImage
./linuxdeploy.AppImage \
    --appdir AppDir \
    --plugin qt \
    --output appimage

popd > /dev/null
echo "--- Success: AppImage successfully generated in ${BUILD_DIR} ---"
