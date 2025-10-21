#!/usr/bin/env python3
import sys
import time
from panda import Panda

def flash_all_pandas():
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

if __name__ == "__main__":
  try:
    success = flash_all_pandas()
    sys.exit(0 if success else 1)
  except Exception as e:
    print(f"Fatal error: {e}")
    sys.exit(1)
