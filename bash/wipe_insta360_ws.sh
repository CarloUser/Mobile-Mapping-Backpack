#!/bin/bash

echo "======================================"
echo "Insta360 ENV setup"
echo "======================================"

SDK_DIR=~/insta360_sdk

# -----------------------------
# Add to bashrc if not exists
# -----------------------------
if ! grep -q "insta360_sdk/lib" ~/.bashrc; then
    echo "export LD_LIBRARY_PATH=$SDK_DIR/lib:\$LD_LIBRARY_PATH" >> ~/.bashrc
    echo "Added LD_LIBRARY_PATH to ~/.bashrc"
else
    echo "LD_LIBRARY_PATH already set"
fi

# Apply immediately
export LD_LIBRARY_PATH=$SDK_DIR/lib:$LD_LIBRARY_PATH

echo "======================================"
echo "ENV setup COMPLETE"
echo "======================================"
