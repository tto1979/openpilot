How to install on Oneplus 3t / EON?
------
1. clone legacypilot to /data/ and make sure it's named openpilot:
   (手動安裝切換至 legacypilot)
```
cd /data/ && rm -fr openpilot ; git clone https://github.com/efiniLan/legacypilot.git openpilot -b master
```

2. run command:
   (在 ssh 畫面下，輸入)
```
cd /data/openpilot && git submodule init && git submodule update
cd /data/openpilot/scripts/ && ./oneplus_update_neos.sh
```

3. Let it download and complete it update, after a couple of reboot, your screen will then stay in fastboot mode.
   (等待下載並讓它重新開機，沒錯誤的話會進入 Android 機器人更新畫面，等自動重新開機)

4. In fastboot mode, select use volume button to select to `Recovery mode` then press power button.
   (在 fastboot 模式，用音量鍵上下選到 Recovery mode 再按下電源鍵)

5. In Recovery mode, tap `apply update` -> `Choose from emulated` -> `0/` -> `update.zip` -> `Reboot system now`
   (在 Recovery mode，點選 `apply update` -> `Choose from emulated` -> `0/` -> `update.zip` -> `Reboot system now`)

6. You should be able to boot into openpilot, if touch screen is not working, try to reboot again.
   (你現在應該可以進入 openpilot 畫面，如果點擊畫面沒有反應，請再重新開機一次)


**You can also switch to other legacypilot based software (such as dragonpilot) now.**
   (你也可以嘗試其它基於 legacypilot 的版本，例如： dragonpilot)
