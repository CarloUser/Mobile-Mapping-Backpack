#!/usr/bin/env bash
# Record the OAK-4D <-> Hesai JT128 pair. Connect both, run this, do the static
# board holds, Ctrl-C to stop. Thin wrapper around record_pair.sh.
exec "$(cd "$(dirname "$0")" && pwd)/record_pair.sh" oak4d "$@"
