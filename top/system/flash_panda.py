#!/usr/bin/env python3
import os
import signal
import subprocess
import sys
import time

from panda import Panda


def stop_pandad():
  """Stop only pandad subprocess, not the entire comma service"""
  try:
    print("Stopping pandad subprocess...")
    result = subprocess.run(
      ["pgrep", "-f", "selfdrive/pandad/pandad"],
      capture_output=True,
      text=True
    )
    
    if result.returncode == 0 and result.stdout.strip():
      pids = result.stdout.strip().split('\n')
      for pid in pids:
        try:
          os.kill(int(pid), signal.SIGINT)
          print(f"Sent SIGINT to pandad process {pid}")
        except ProcessLookupError:
          pass
      
      time.sleep(3)
      print("Pandad stopped successfully")
      return True
    else:
      print("Pandad not running")
      return False
      
  except Exception as e:
    print(f"Warning: Failed to stop pandad: {e}")
    return False


def wait_for_pandad_exit():
  """Wait for pandad to fully exit"""
  max_wait = 10
  while max_wait > 0:
    result = subprocess.run(
      ["pgrep", "-f", "selfdrive/pandad/pandad"],
      capture_output=True
    )
    if result.returncode != 0:
      return True
    time.sleep(1)
    max_wait -= 1
  return False


def flash_all_pandas():
  # Stop only pandad, not the entire comma service
  pandad_was_stopped = stop_pandad()

  if pandad_was_stopped:
    if not wait_for_pandad_exit():
      print("Warning: pandad may still be running")
  
  try:
    serials = Panda.list()

    if not serials:
      print("No Panda devices found!")
      return False

    print(f"Found {len(serials)} Panda device(s): {serials}")

    success_count = 0
    for serial in serials:
      try:
        print(f"\n{'='*50}")
        print(f"Flashing Panda: {serial}")
        print(f"{'='*50}")

        panda = Panda(serial)
        print("Resetting into bootstub mode...")
        panda.reset(enter_bootstub=True)
        time.sleep(2)

        print("Flashing firmware...")
        panda = Panda(serial)
        panda.flash()

        print(f"Successfully flashed {serial}")
        panda.close()
        success_count += 1

      except Exception as e:
        print(f"Error flashing {serial}: {e}")
        continue

    print(f"\n{'='*50}")
    print(f"Flashing complete: {success_count}/{len(serials)} successful")
    print(f"{'='*50}")

    return success_count == len(serials)

  except Exception as e:
    print(f"Error during flash: {e}")
    return False


if __name__ == "__main__":
  try:
    success = flash_all_pandas()
    # Don't restart pandad - manager will handle it
    print("\nFlash completed. System will reboot...")
    sys.exit(0 if success else 1)
  except Exception as e:
    print(f"Fatal error: {e}")
    sys.exit(1)
