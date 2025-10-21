#!/usr/bin/env python3
import subprocess
import sys
import time
from panda import Panda


def stop_pandad():
  """Stop pandad service to release USB"""
  try:
    print("Stopping pandad service...")
    subprocess.run(["sudo", "systemctl", "stop", "comma"], check=True, timeout=10)
    time.sleep(3)
    print("Pandad stopped successfully")
    return True
  except Exception as e:
    print(f"Warning: Failed to stop pandad: {e}")
    return False


def start_pandad():
  """Start pandad service"""
  try:
    print("Starting pandad service...")
    subprocess.run(["sudo", "systemctl", "start", "comma"], check=True, timeout=10)
    time.sleep(3)
    print("Pandad started successfully")
    return True
  except Exception as e:
    print(f"Error: Failed to start pandad: {e}")
    return False


def flash_all_pandas():
  # Stop pandad first
  pandad_was_stopped = stop_pandad()
  
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

  finally:
    # Always restart pandad
    if pandad_was_stopped:
      start_pandad()


if __name__ == "__main__":
  try:
    success = flash_all_pandas()
    sys.exit(0 if success else 1)
  except Exception as e:
    print(f"Fatal error: {e}")
    # Ensure pandad is restarted
    start_pandad()
    sys.exit(1)
