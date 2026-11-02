#!/usr/bin/env bash
# Shared deployment settings. Source from launchers; local overrides stay out of Git.
_deployment_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -f "$_deployment_repo/config/deployment.local.env" ]]; then
    source "$_deployment_repo/config/deployment.local.env"
fi
: "${JETSON_HOST:=192.0.2.10}"
: "${JETSON_USER:=ubuntu}"
: "${WORKSTATION_HOST:=192.0.2.86}"
: "${ISAAC_HOST:=192.0.2.33}"
: "${ISAAC_USER:=ubuntu}"
: "${JETSON_SSH_TARGET:=$JETSON_USER@$JETSON_HOST}"
: "${ISAAC_SSH_TARGET:=$ISAAC_USER@$ISAAC_HOST}"
if [[ -f "$_deployment_repo/config/calibration_dds.local.xml" ]]; then
    : "${ROBOMASTER_DDS_PROFILE:=$_deployment_repo/config/calibration_dds.local.xml}"
else
    : "${ROBOMASTER_DDS_PROFILE:=$_deployment_repo/config/calibration_dds.xml}"
fi
export JETSON_HOST JETSON_USER JETSON_SSH_TARGET ISAAC_HOST ISAAC_USER ISAAC_SSH_TARGET
export ROBOMASTER_DDS_PROFILE
unset _deployment_repo
