#!/usr/bin/env bash
# Panda Flash Verification Tool
# Verifies that Panda firmware has been successfully flashed

echo "╔════════════════════════════════════════╗"
echo "║   Panda Flash Verification Tool        ║"
echo "╚════════════════════════════════════════╝"
echo ""

cd /data/openpilot

# Run all tests in a single Python process
python3 << 'PYEOF'
import os
import sys

from panda import Panda
from openpilot.common.params import Params


def test_bootstub():
    """Test 1: Check if Panda is in bootstub mode"""
    try:
        pandas = Panda.list()

        if not pandas:
            return {"status": "NO_DEVICE", "passed": False, "message": "No Panda devices found"}

        for serial in pandas:
            panda = Panda(serial)
            if panda.bootstub:
                panda.close()
                return {"status": "FAIL", "passed": False, "message": "Some Pandas stuck in bootstub mode"}
            panda.close()

        return {"status": "PASS", "passed": True, "message": "All Pandas out of bootstub mode"}

    except Exception as e:
        return {"status": "ERROR", "passed": False, "message": f"Error: {e}"}


def test_firmware_version():
    """Test 2: Check firmware versions"""
    try:
        pandas = Panda.list()

        if not pandas:
            return {"passed": False, "versions": [], "message": "No Panda devices found"}

        versions = []
        for serial in pandas:
            panda = Panda(serial)
            version = panda.get_version()
            versions.append({"serial": serial[:8], "version": version})
            panda.close()

        return {"passed": True, "versions": versions, "message": "Firmware versions retrieved"}

    except Exception as e:
        return {"passed": False, "versions": [], "message": f"Error: {e}"}


def test_flash_log():
    """Test 3: Check flash log"""
    log_path = "/tmp/flash_panda.log"

    if not os.path.exists(log_path):
        return {"passed": False, "message": "No flash log found (may not have flashed yet)"}

    try:
        with open(log_path, 'r') as f:
            content = f.read()
            if "successful" in content:
                return {"passed": True, "message": "Flash log shows success"}
            else:
                return {"passed": False, "message": "Flash log exists but no success message"}
    except Exception as e:
        return {"passed": False, "message": f"Error reading log: {e}"}


def test_panda_signatures():
    """Test 4: Check stored PandaSignatures"""
    try:
        params = Params()
        sigs = params.get("PandaSignatures", encoding='utf-8')

        if sigs:
            return {"passed": True, "signatures": sigs[:50], "message": "PandaSignatures stored"}
        else:
            return {"passed": False, "signatures": "", "message": "No PandaSignatures found"}

    except Exception as e:
        return {"passed": False, "signatures": "", "message": f"Error: {e}"}


# Run all tests
print("[Test 1/4] Checking Bootstub Status...")
result1 = test_bootstub()
if result1["passed"]:
    print(f"  ✅ PASS: {result1['message']}")
elif result1["status"] == "NO_DEVICE":
    print(f"  ⚠️  ERROR: {result1['message']}")
else:
    print(f"  ❌ FAIL: {result1['message']}")

print("\n[Test 2/4] Checking Firmware Version...")
result2 = test_firmware_version()
if result2["passed"]:
    for v in result2["versions"]:
        print(f"  Panda {v['serial']}... : {v['version']}")
else:
    print(f"  ⚠️  {result2['message']}")

print("\n[Test 3/4] Checking Flash Log...")
result3 = test_flash_log()
if result3["passed"]:
    print(f"  ✅ PASS: {result3['message']}")
else:
    print(f"  ⚠️  WARNING: {result3['message']}")

print("\n[Test 4/4] Checking Stored Signatures...")
result4 = test_panda_signatures()
if result4["passed"]:
    print(f"  ✅ PASS: {result4['message']}")
    print(f"  Signatures: {result4['signatures']}...")
else:
    print(f"  ⚠️  WARNING: {result4['message']}")

# Summary
print("\n╔════════════════════════════════════════╗")
print("║           TEST SUMMARY                 ║")
print("╚════════════════════════════════════════╝")

total = sum([result1["passed"], result2["passed"], result3["passed"], result4["passed"]])
print(f"Tests Passed: {total}/4")
print("")

if result1["passed"]:
    print("✅ Panda is NOT in bootstub mode - Flash successful!")
    print("")

    if total == 4:
        print("🎉 Perfect! All tests passed!")
    elif total >= 2:
        print("⚠️  Flash succeeded but some secondary checks failed")

    sys.exit(0)
else:
    print("❌ Panda IS in bootstub mode - Flash may have failed!")
    print("")
    print("Recommended actions:")
    print("1. Run flash again through UI")
    print("2. Manually recover: python3 -c 'from panda import Panda; Panda().recover()'")
    print("3. Check USB connection")
    print("4. Check logs: cat /tmp/flash_panda.log")
    sys.exit(1)
PYEOF
