#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

source "$DIR/launch_env.sh"

function agnos_init {
  # TODO: move this to agnos
  sudo rm -f /data/etc/NetworkManager/system-connections/*.nmmeta

  # set success flag for current boot slot
  sudo abctl --set_success

  # TODO: do this without udev in AGNOS
  # udev does this, but sometimes we startup faster
  sudo chgrp gpu /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0
  sudo chmod 660 /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0

  # Check if AGNOS update is required
  if [ $(< /VERSION) != "$AGNOS_VERSION" ]; then
    AGNOS_PY="$DIR/system/hardware/tici/agnos.py"
    MANIFEST="$DIR/system/hardware/tici/agnos.json"
    if $AGNOS_PY --verify $MANIFEST; then
      sudo reboot
    fi
    $DIR/system/hardware/tici/updater $AGNOS_PY $MANIFEST
  fi
}

# comma three (tici) family only: pick the panda stack by the internal panda's MCU.
#   F4 (dos, the stock C3 panda) -> TICI_DOS=1: panda_tici + selfdrive/pandad/pandad_tici
#   H7 (tres)                    -> TICI_TRES=1: upstream panda + selfdrive/pandad
# comma 3X is never touched. The MCU type is a hardware fact, so it's cached in /data
# (survives branch/fork switches, re-detected after a reset). If detection fails,
# fall back to dos (pandad_tici can flash both F4 and H7) without caching.
# Based on dragonpilot's set_tici_hw (credit: Rick Lan).
function set_tici_hw {
  grep -q "tici" /sys/firmware/devicetree/base/model 2>/dev/null || return 0
  export TICI_HW=1

  local cache="/data/tici_panda_mcu"
  local attempts=15 confirm=3
  local mcu="" count=0 last="" cur=""

  case "$(cat $cache 2>/dev/null)" in
    F4|H7) mcu="$(cat $cache)"; echo "panda MCU $mcu [cached]" ;;
  esac

  if [ -z "$mcu" ]; then
    echo "Querying panda MCU type..."
    for attempt in $(seq 1 $attempts); do
      # wait longer while the panda is still enumerating, short between confirmations
      if [ -n "$last" ]; then sleep 1; else sleep 3; fi

      # only the internal panda is visible here: the aux USB-C port is still in device mode
      case "$(python -c "from panda_tici import Panda; p = Panda(cli=False); print(p.get_mcu_type()); p.close()" 2>/dev/null)" in
        *McuType.F4*) cur="F4" ;;
        *McuType.H7*) cur="H7" ;;
        *)            cur="" ;;
      esac

      if [ -n "$cur" ] && [ "$cur" = "$last" ]; then
        count=$((count + 1))
      else
        count=1
        last="$cur"
      fi

      if [ -n "$cur" ] && [ $count -ge $confirm ]; then
        mcu="$cur"
        { echo "$mcu" > $cache; } 2>/dev/null
        break
      fi
      echo "panda MCU read='${cur:-UNKNOWN}' (confirmed $count/$confirm, attempt $attempt/$attempts)"
    done
  fi

  if [ -z "$mcu" ]; then
    echo "panda MCU unknown after $attempts attempts, assuming dos (F4)"
    mcu="F4"
  fi

  if [ "$mcu" = "F4" ]; then
    echo "TICI (DOS) detected"
    export TICI_DOS=1
    set_aux_panda
  else
    echo "TICI (TRES) detected"
    export TICI_TRES=1
  fi
}

# The aux USB-C port boots in OTG idle ("none"); a 2nd (external) panda there only enumerates
# once the port is in host mode. Keep host mode only if a 2nd panda shows up, otherwise revert
# so the port still works as a USB device (PC connect). Probed every boot (plug/unplug).
# From dragonpilot (credit: Rick Lan).
function set_aux_panda {
  local mode="/sys/devices/platform/soc/a600000.ssusb/mode"
  [ -e "$mode" ] || return 0

  echo "Checking for aux panda (switching USB-C port to host mode)..."
  echo host | sudo tee "$mode" > /dev/null 2>&1
  for i in $(seq 1 6); do
    sleep 0.5
    if [ "$(lsusb 2>/dev/null | grep -c 'comma.ai panda')" -ge 2 ]; then
      echo "aux panda detected (USB host mode kept)"
      return 0
    fi
  done

  echo "no aux panda found; reverting USB-C port to device mode"
  echo none | sudo tee "$mode" > /dev/null 2>&1
}

# comma three (tici) family only: a unit without the amplifier at i2c 0x10
# (e.g. O3 Lite) has no speaker, driver camera or modem
function set_lite_hw {
  grep -q "tici" /sys/firmware/devicetree/base/model 2>/dev/null || return 0

  if [ -z "$(i2cget -y 0 0x10 0x00 2>/dev/null)" ]; then
    echo "Lite HW"
    export LITE=1
    export DISABLE_DRIVER=1
  fi
}

# comma three (tici) family only: AGNOS 13+ dropped the NVMe from fstab,
# so mount it where the AGNOS 12 fstab did (/data/media)
function mount_nvme {
  grep -q "tici" /sys/firmware/devicetree/base/model 2>/dev/null || return 0
  [ -n "$LITE" ] && return 0
  [ -f /persist/comma/living-in-the-moment ] && return 0

  local dev=/dev/nvme0n1
  for i in $(seq 1 5); do
    [ -b $dev ] && break
    sleep 1
  done
  if [ ! -b $dev ]; then
    echo "NVMe not found"
    return 0
  fi
  [ -b ${dev}p1 ] && dev=${dev}p1

  sudo mkdir -p /data/media
  if ! mountpoint -q /data/media; then
    sudo mount -o discard,nosuid,nodev $dev /data/media || echo "NVMe mount failed"
  fi

  if mountpoint -q /data/media; then
    sudo mkdir -p /data/media/0/realdata
    sudo chown comma:comma /data/media/0 /data/media/0/realdata
  fi
}

function launch {
  # Remove orphaned git lock if it exists on boot
  [ -f "$DIR/.git/index.lock" ] && rm -f $DIR/.git/index.lock

  # Check to see if there's a valid overlay-based update available. Conditions
  # are as follows:
  #
  # 1. The DIR init file has to exist, with a newer modtime than anything in
  #    the DIR Git repo. This checks for local development work or the user
  #    switching branches/forks, which should not be overwritten.
  # 2. The FINALIZED consistent file has to exist, indicating there's an update
  #    that completed successfully and synced to disk.

  if [ -f "${DIR}/.overlay_init" ]; then
    find ${DIR}/.git -newer ${DIR}/.overlay_init | grep -q '.' 2> /dev/null
    if [ $? -eq 0 ]; then
      echo "${DIR} has been modified, skipping overlay update installation"
    else
      if [ -f "${STAGING_ROOT}/finalized/.overlay_consistent" ]; then
        if [ ! -d /data/safe_staging/old_openpilot ]; then
          echo "Valid overlay update found, installing"
          LAUNCHER_LOCATION="${BASH_SOURCE[0]}"

          mv $DIR /data/safe_staging/old_openpilot
          mv "${STAGING_ROOT}/finalized" $DIR
          cd $DIR

          echo "Restarting launch script ${LAUNCHER_LOCATION}"
          unset AGNOS_VERSION
          exec "${LAUNCHER_LOCATION}"
        else
          echo "openpilot backup found, not updating"
          # TODO: restore backup? This means the updater didn't start after swapping
        fi
      fi
    fi
  fi

  # handle pythonpath
  ln -sfn $(pwd) /data/pythonpath
  export PYTHONPATH="$PWD"

  # hardware specific init
  if [ -f /AGNOS ]; then
    set_tici_hw
    set_lite_hw
    mount_nvme
    agnos_init
  fi

  # write tmux scrollback to a file
  tmux capture-pane -pq -S-1000 > /tmp/launch_log

  python ./opendbc_repo/opendbc/car/fingerprints.py > /data/openpilot/selfdrive/car/top_tmp/AllCars

  python ./force_car_recognition.py

  cat /etc/resolv.conf
  echo "nameserver 8.8.8.8" | sudo tee /etc/resolv.conf
  echo "nameserver 8.8.4.4" | sudo tee -a /etc/resolv.conf
  sudo systemctl restart network-manager
  sudo apt update
  sudo $(which pip3) install --upgrade pip
  sudo $(which pip3) install flask

  # start manager
  cd system/manager
  if [ ! -f $DIR/prebuilt ]; then
    ./build.py
  fi
  ./manager.py

  # if broken, keep on screen error
  while true; do sleep 1; done
}

launch
