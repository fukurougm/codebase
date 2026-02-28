#!/usr/bin/env bash
set -euo pipefail

PROGNAME="$(basename "$0")"
REPO="fukurougm/codebase/ros2dev"
NO_VERIFY=0

usage() {
  cat <<EOF
Usage: $PROGNAME [options] [TAG] [ARCH]

Install ros2dev from GitHub Releases.

Positional arguments:
  TAG         Release tag (e.g., v0.1.0). If omitted, the latest release is used.
  ARCH        Architecture: x86_64 or aarch64. If omitted, detected from uname -m.

Options:
  -n          Skip checksum verification (not recommended)
  -h          Show this help

Examples:
  # install latest for current arch
  $PROGNAME

  # install specific tag for aarch64
  $PROGNAME v0.1.0 aarch64

  # install but skip checksum verification
  $PROGNAME -n v0.1.0
EOF
}

while getopts ":nh" opt; do
  case $opt in
    n) NO_VERIFY=1 ;;
    h) usage; exit 0 ;;
    *) usage; exit 1 ;;
  esac
done
shift $((OPTIND-1))

TAG="${1-}"
ARCH="${2-}"

# detect arch if not given
if [ -z "$ARCH" ]; then
  case "$(uname -m)" in
    x86_64) ARCH=x86_64 ;;
    aarch64|arm64) ARCH=aarch64 ;;
    *) echo "Unsupported architecture: $(uname -m)" >&2; exit 1 ;;
  esac
fi

# detect latest tag if not given
if [ -z "$TAG" ]; then
  echo "Detecting latest release tag from GitHub..."
  TAG=$(curl -sSf "https://api.github.com/repos/${REPO}/releases/latest" \
    | grep -Po '"tag_name":\s*"\K(.*?)(?=")' || true)
  if [ -z "$TAG" ]; then
    echo "Failed to detect latest release. Please provide a TAG." >&2
    exit 1
  fi
fi

ARTIFACT="ros2dev-linux-${ARCH}.tar.gz"
BASEURL="https://github.com/${REPO}/releases/download/${TAG}"
TMPDIR="$(mktemp -d)"
cleanup() { rm -rf "$TMPDIR"; }
trap cleanup EXIT

DL="$TMPDIR/$ARTIFACT"
SHAFILE="$TMPDIR/SHA256SUMS.txt"

echo "Downloading ${ARTIFACT} from ${TAG}..."
if ! curl -L -f -o "$DL" "$BASEURL/$ARTIFACT"; then
  echo "Failed downloading $BASEURL/$ARTIFACT" >&2; exit 1
fi

echo "Downloading SHA256SUMS.txt..."
# don't fail if SHA doesn't exist (we'll handle it below)
curl -L -f -o "$SHAFILE" "$BASEURL/SHA256SUMS.txt" || true

if [ "$NO_VERIFY" -eq 0 ]; then
  if [ -f "$SHAFILE" ] && grep -q " $ARTIFACT" "$SHAFILE"; then
    expected=$(grep " $ARTIFACT" "$SHAFILE" | awk '{print $1}')
    actual=$(sha256sum "$DL" | awk '{print $1}')
    if [ "$expected" != "$actual" ]; then
      echo "Checksum mismatch! Aborting." >&2
      exit 1
    else
      echo "Checksum OK"
    fi
  else
    echo "No checksum found for $ARTIFACT in SHA256SUMS.txt." >&2
    echo "Re-run with -n to skip verification if you trust the release." >&2
    exit 1
  fi
else
  echo "Skipping checksum verification as requested (-n)."
fi

# Extract and find the binary
echo "Extracting $ARTIFACT..."
tar -xzf "$DL" -C "$TMPDIR"
# The archive should contain a single binary, e.g. ros2dev-linux-x86_64
BIN_CANDIDATE="$TMPDIR/ros2dev-linux-${ARCH}"
if [ -f "$BIN_CANDIDATE" ]; then
  BIN_PATH="$BIN_CANDIDATE"
else
  # fallback: find first regular file in the archive
  BIN_IN_TAR=$(tar -tzf "$DL" | grep -E '^[^/]+$' | head -n1)
  if [ -z "$BIN_IN_TAR" ]; then
    echo "Could not locate binary inside archive." >&2; exit 1
  fi
  tar -xzf "$DL" -C "$TMPDIR" "$BIN_IN_TAR"
  BIN_PATH="$TMPDIR/$BIN_IN_TAR"
fi

if [ ! -f "$BIN_PATH" ]; then
  echo "Extracted binary not found: $BIN_PATH" >&2; exit 1
fi

# Install to /usr/local/bin/ros2dev
echo "Installing to /usr/local/bin/ros2dev (sudo may be required)..."
sudo install -m 0755 "$BIN_PATH" /usr/local/bin/ros2dev

echo "Installation complete. Run 'ros2dev --help' to verify."
exit 0
