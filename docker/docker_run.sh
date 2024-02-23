#!/bin/bash

set -e

##### start: command line argument parse

pos_args=()

while [[ $# -gt 0 ]]; do
  key="$1"
  case ${key} in
  --force)
    opt_arg_force="YES"
    shift
    ;;
  --no-cache)
    opt_no_cache="YES"
    shift
    ;;
  --run)
    opt_arg_run="YES"
    shift
    ;;
  --image-name)
    opt_arg_image_name="$2"
    shift
    shift
    ;;
  --container-name)
    opt_arg_container_name="$2"
    shift
    shift
    ;;
  -*)
    echo "Unknown option ${key}"
    exit 1
    ;;
  *)
    pos_args+=("$1")
    shift
    ;;
  esac
done

##### end: command line argument parse


random_string=$(echo ${RANDOM} | md5sum | head -c 7)
image_name="${opt_arg_image_name:-plusai}"
container_name="${opt_arg_container_name:-${image_name}_${random_string}_${USER}}"

if [[ -z $(which docker) ]]; then
  echo "You have not installed Docker!"
  exit 1
fi

if [[ -z $(docker image ls | grep "${image_name}") ]]; then
  echo "Docker image ${image_name} is not found, building ..."
  docker build --file docker/Dockerfile --tag "${image_name}" .
elif [[ "${opt_arg_force}" == "YES" ]]; then
  if [[ "${opt_no_cache}" == "YES" ]]; then
    echo "Force rebuild Docker image ${image_name} ..."
    docker build --no-cache --file docker/Dockerfile --tag "${image_name}" .
  else
    echo "Force build Docker image ${image_name} ..."
    docker build --file docker/Dockerfile --tag "${image_name}" .
  fi
else
  echo "Using existing Docker image ${image_name}"
fi

if [[ "${opt_arg_run}" == "YES" ]]; then
  echo "Running container ${container_name} on image ${image_name}"

  pwd=$(pwd)
  cols=$(tput cols)
  lines=$(tput lines)

  plusai_config_dir="${HOME}/.config/plusai/"
  mkdir -p "${plusai_config_dir}"

  # Setup XAUTH
  xauth_file="${plusai_config_dir}/docker.${random_string}.xauth"
  xauth_tmp_file="${plusai_config_dir}/docker.${random_string}.tmp"

  touch "${xauth_file}"
  xauth nlist "${DISPLAY}" | head -c -1 > "${xauth_tmp_file}"
  xauth -f "${xauth_file}" nmerge "${xauth_tmp_file}"
  rm "${xauth_tmp_file}"

  # Find video device
  video_device=""
  for file in /dev/dri/*; do
    if [[ "${file}" =~ ^/dev/dri/card[0-9]$ ]]; then
      video_device="${file}"
    fi
  done

  docker run \
    --runtime nvidia \
    --ipc host \
    --network host \
    --name "${container_name}" \
    -i \
    -t \
    --privileged \
    --cap-add sys_ptrace \
    -u 1000:1000 \
    --group-add sudo \
    --group-add audio \
    -e "DOCKER_IMAGE=${image_name}" \
    -e "USER=${USER}" \
    -e "DISPLAY=${DISPLAY}" \
    -e "COLUMNS=${cols}" \
    -e "LINES=${lines}" \
    -e "TERM=xterm-256color" \
    -e "COLORTERM=truecolor" \
    -e "LC_ALL=" \
    -e "SSH_AUTH_SOCK=/tmp/ssh-agent" \
    -e "PULSE_SERVER=unix:/tmp/pulse/native" \
    -e "XSOCK=/tmp/.X11-unix" \
    -e "XAUTHORITY=${xauth_file}" \
    -e "CONTAINER=${container_name}" \
    -v "/etc/hosts:/etc/hosts:ro" \
    -v "/etc/resolv.conf:/etc/resolv.conf:ro" \
    -v "/etc/localtime:/etc/localtime:ro" \
    -v "/etc/timezone:/etc/timezone:ro" \
    -v "/etc/passwd:/etc/passwd:ro" \
    -v "/etc/group:/etc/group:ro" \
    -v "/etc/shadow:/etc/shadow:ro" \
    -v "/etc/sudoers.d:/etc/sudoers.d:ro" \
    -v "/etc/sudoers:/etc/sudoers:ro" \
    -v "/dev/snd:/dev/snd:rw" \
    -v "/media:/media:rw" \
    -v "/var/crash:/var/crash:rw" \
    -v "/run/user/1000/keyring/ssh:/tmp/ssh-agent:rw" \
    -v "/run/user/1000/pulse:/tmp/pulse:rw" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v "${xauth_file}:${xauth_file}:rw" \
    -v "${HOME}:${HOME}:rw" \
    --device "${video_device}" \
    --workdir "${pwd}" \
    "${image_name}" \
    /bin/bash

fi
