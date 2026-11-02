#!/usr/bin/env bash
# Generate an ignored DDS profile without putting LAN addresses in tracked files.
set -euo pipefail
deployment_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f "$deployment_repo/config/deployment.local.env" ]]; then
    echo 'Copy config/deployment.example.env to config/deployment.local.env and edit it first.' >&2
    exit 1
fi
source "$deployment_repo/scripts/deployment_env.sh"
for address in "$JETSON_HOST" "$WORKSTATION_HOST"; do
    [[ "$address" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]] || {
        echo 'DDS peer addresses must be IPv4 literals.' >&2; exit 1;
    }
    IFS=. read -r -a octets <<< "$address"
    for octet in "${octets[@]}"; do
        (( 10#$octet <= 255 )) || { echo 'Invalid IPv4 address.' >&2; exit 1; }
    done
    [[ "$address" != 192.0.2.* ]] || {
        echo 'Replace the documentation addresses with your actual LAN addresses first.' >&2; exit 1;
    }
done
sed -e "s/192\.0\.2\.10/$JETSON_HOST/g" \
    -e "s/192\.0\.2\.86/$WORKSTATION_HOST/g" \
    "$deployment_repo/config/calibration_dds.xml" > "$deployment_repo/config/calibration_dds.local.xml"
echo 'Created config/calibration_dds.local.xml. Keep it and deployment.local.env private.'
