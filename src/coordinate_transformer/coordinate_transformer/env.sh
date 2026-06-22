export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v '^/opt/MVS/lib/aarch64$' | paste -sd:)
