#!/usr/bin/env bash
# Keep the RoboMaster SocketCAN interface configured after disconnects/bus-off.
#
# The SH-C31A gs_usb driver does not support restart-ms, so recovery is done by
# explicitly cycling the interface down and up.

set -uo pipefail

CAN_INTERFACE="${CAN_INTERFACE:-can0}"
CAN_BITRATE="${CAN_BITRATE:-1000000}"
CAN_TXQUEUELEN="${CAN_TXQUEUELEN:-1000}"
POLL_INTERVAL="${POLL_INTERVAL:-1}"
CAN_REBIND_AFTER_FAILURES="${CAN_REBIND_AFTER_FAILURES:-3}"
CAN_REBIND_COOLDOWN="${CAN_REBIND_COOLDOWN:-30}"
RUN_ONCE=false

usage() {
    echo "Usage: $0 [--interface can0] [--bitrate 1000000] [--txqueuelen 1000] [--interval 1] [--once]"
}

while (($#)); do
    case "$1" in
        --interface)
            CAN_INTERFACE="${2:?--interface requires a value}"
            shift 2
            ;;
        --bitrate)
            CAN_BITRATE="${2:?--bitrate requires a value}"
            shift 2
            ;;
        --txqueuelen)
            CAN_TXQUEUELEN="${2:?--txqueuelen requires a value}"
            shift 2
            ;;
        --interval)
            POLL_INTERVAL="${2:?--interval requires a value}"
            shift 2
            ;;
        --once)
            RUN_ONCE=true
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if [[ ! "$CAN_BITRATE" =~ ^[0-9]+$ ]] || ((CAN_BITRATE <= 0)); then
    echo "Invalid bitrate: $CAN_BITRATE" >&2
    exit 2
fi
if [[ ! "$CAN_TXQUEUELEN" =~ ^[0-9]+$ ]] || ((CAN_TXQUEUELEN <= 0)); then
    echo "Invalid txqueuelen: $CAN_TXQUEUELEN" >&2
    exit 2
fi
if [[ ! "$POLL_INTERVAL" =~ ^[0-9]+([.][0-9]+)?$ || "$POLL_INTERVAL" =~ ^0+([.]0+)?$ ]]; then
    echo "Invalid polling interval: $POLL_INTERVAL" >&2
    exit 2
fi
if [[ ! "$CAN_REBIND_AFTER_FAILURES" =~ ^[0-9]+$ ]] || ((CAN_REBIND_AFTER_FAILURES <= 0)); then
    echo "Invalid gs_usb rebind failure threshold: $CAN_REBIND_AFTER_FAILURES" >&2
    exit 2
fi
if [[ ! "$CAN_REBIND_COOLDOWN" =~ ^[0-9]+$ ]] || ((CAN_REBIND_COOLDOWN <= 0)); then
    echo "Invalid gs_usb rebind cooldown: $CAN_REBIND_COOLDOWN" >&2
    exit 2
fi
if ! command -v ip >/dev/null 2>&1; then
    echo "The 'ip' command is required (package: iproute2)." >&2
    exit 1
fi
if ! command -v tc >/dev/null 2>&1; then
    echo "The 'tc' command is required (package: iproute2)." >&2
    exit 1
fi

if ((EUID != 0)); then
    sudo_args=(
        --interface "$CAN_INTERFACE"
        --bitrate "$CAN_BITRATE"
        --txqueuelen "$CAN_TXQUEUELEN"
        --interval "$POLL_INTERVAL"
    )
    if [[ "$RUN_ONCE" == true ]]; then
        sudo_args+=(--once)
    fi
    exec sudo -- "$0" "${sudo_args[@]}"
fi

log() {
    echo "$(date --iso-8601=seconds) $*"
}

configure_can() {
    # Ignore failure of the down operation: a newly attached device is already down.
    ip link set dev "$CAN_INTERFACE" down 2>/dev/null || true
    if ! ip link set dev "$CAN_INTERFACE" txqueuelen "$CAN_TXQUEUELEN"; then
        log "Failed to set $CAN_INTERFACE txqueuelen to $CAN_TXQUEUELEN."
        return 1
    fi
    if ! ip link set dev "$CAN_INTERFACE" type can bitrate "$CAN_BITRATE"; then
        log "Failed to set $CAN_INTERFACE bitrate to $CAN_BITRATE."
        return 1
    fi
    if ! ip link set dev "$CAN_INTERFACE" up; then
        log "Failed to bring $CAN_INTERFACE up."
        return 1
    fi
}

rebind_gs_usb() {
    local device_path driver_path usb_interface attempt

    if ! device_path="$(readlink -f "/sys/class/net/$CAN_INTERFACE/device" 2>/dev/null)"; then
        log "Cannot rebind $CAN_INTERFACE: its USB device path is unavailable." >&2
        return 1
    fi
    if ! driver_path="$(readlink -f "$device_path/driver" 2>/dev/null)"; then
        log "Cannot rebind $CAN_INTERFACE: its driver path is unavailable." >&2
        return 1
    fi
    if [[ "$driver_path" != "/sys/bus/usb/drivers/gs_usb" ]]; then
        log "Refusing to rebind $CAN_INTERFACE: driver is ${driver_path##*/}, not gs_usb." >&2
        return 1
    fi

    usb_interface="${device_path##*/}"
    if [[ ! "$usb_interface" =~ ^[0-9]+-[0-9]+([.][0-9]+)*:[0-9]+[.][0-9]+$ ]]; then
        log "Refusing to rebind $CAN_INTERFACE: unexpected USB interface name '$usb_interface'." >&2
        return 1
    fi

    log "Rebinding gs_usb interface $usb_interface after repeated $CAN_INTERFACE configuration failures."
    if ! printf '%s' "$usb_interface" > /sys/bus/usb/drivers/gs_usb/unbind; then
        log "Failed to unbind gs_usb interface $usb_interface." >&2
        return 1
    fi
    sleep 1
    if ! printf '%s' "$usb_interface" > /sys/bus/usb/drivers/gs_usb/bind; then
        log "Failed to bind gs_usb interface $usb_interface." >&2
        return 1
    fi

    for ((attempt = 0; attempt < 20; attempt++)); do
        if [[ -e "/sys/class/net/$CAN_INTERFACE" ]]; then
            log "$CAN_INTERFACE reappeared after the gs_usb rebind."
            return 0
        fi
        sleep 0.25
    done

    log "$CAN_INTERFACE did not reappear after rebinding gs_usb interface $usb_interface." >&2
    return 1
}

configuration_failures=0
last_rebind_attempt=$((-CAN_REBIND_COOLDOWN))

record_configuration_failure() {
    ((configuration_failures += 1))
    if ((configuration_failures < CAN_REBIND_AFTER_FAILURES)); then
        return
    fi
    if ((SECONDS - last_rebind_attempt < CAN_REBIND_COOLDOWN)); then
        return
    fi

    last_rebind_attempt=$SECONDS
    if rebind_gs_usb; then
        configuration_failures=0
    else
        log "Automatic gs_usb rebind failed; normal recovery will continue." >&2
    fi
}

record_configuration_success() {
    configuration_failures=0
}

queue_backlog_packets() {
    local queue_details
    if ! queue_details="$(tc -s qdisc show dev "$CAN_INTERFACE" 2>/dev/null)"; then
        echo 0
        return
    fi
    if [[ "$queue_details" =~ backlog[[:space:]]+[0-9]+b[[:space:]]+([0-9]+)p ]]; then
        echo "${BASH_REMATCH[1]}"
    else
        echo 0
    fi
}

# gs_usb can remain ERROR-ACTIVE while an unacknowledged transmit queue is
# effectively wedged. A healthy 1 Mbps bus drains far below this threshold.
queue_stall_threshold=$(((CAN_TXQUEUELEN * 9 + 9) / 10))

last_status=""
while true; do
    if ! details="$(ip -details link show dev "$CAN_INTERFACE" 2>/dev/null)"; then
        status="missing"
        if [[ "$status" != "$last_status" ]]; then
            log "$CAN_INTERFACE is missing; waiting for the USB CAN adapter."
        fi
        result=1
    elif [[ "$details" == *"BUS-OFF"* || "$details" == *"STOPPED"* ]]; then
        status="recovering"
        log "$CAN_INTERFACE is bus-off/stopped; cycling it down and up."
        if configure_can; then
            record_configuration_success
            log "$CAN_INTERFACE restored at ${CAN_BITRATE} bit/s (txqueuelen $CAN_TXQUEUELEN)."
            result=0
        else
            log "Failed to restore $CAN_INTERFACE; retrying." >&2
            record_configuration_failure
            result=1
        fi
    elif backlog_packets="$(queue_backlog_packets)" && ((backlog_packets >= queue_stall_threshold)); then
        status="queue-stalled"
        log "$CAN_INTERFACE transmit queue is stalled (${backlog_packets}/${CAN_TXQUEUELEN} packets); cycling it down and up."
        if configure_can; then
            record_configuration_success
            log "$CAN_INTERFACE transmit queue cleared at ${CAN_BITRATE} bit/s."
            result=0
        else
            log "Failed to clear the $CAN_INTERFACE transmit queue; retrying." >&2
            record_configuration_failure
            result=1
        fi
    else
        first_line="${details%%$'\n'*}"
        link_is_up=false
        if [[ "$first_line" == *"<UP,"* || "$first_line" == *",UP,"* || "$first_line" == *",UP>"* ]]; then
            link_is_up=true
        fi
        if [[ "$details" != *"bitrate $CAN_BITRATE"* || "$link_is_up" != true ]]; then
            status="configuring"
            log "$CAN_INTERFACE exists but is down or incorrectly configured; configuring it."
            if configure_can; then
                record_configuration_success
                log "$CAN_INTERFACE ready at ${CAN_BITRATE} bit/s (txqueuelen $CAN_TXQUEUELEN)."
                result=0
            else
                log "Failed to configure $CAN_INTERFACE; retrying." >&2
                record_configuration_failure
                result=1
            fi
        else
            status="ready"
            record_configuration_success
            if [[ "$status" != "$last_status" ]]; then
                log "$CAN_INTERFACE is ready at ${CAN_BITRATE} bit/s."
            fi
            result=0
        fi
    fi

    last_status="$status"
    if [[ "$RUN_ONCE" == true ]]; then
        exit "$result"
    fi
    sleep "$POLL_INTERVAL"
done
