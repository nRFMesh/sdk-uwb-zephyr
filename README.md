# sdk-uwb-zephyr
Zephyr sdk for uwb module DWM1001C based on nRF52832 and DW1000 Transceiver
# discussions
get support, give feedback or simply chat to brainstorm about ideas on the forum

https://github.com/nRFMesh/sdk-uwb-zephyr/discussions
# Framework doc
https://www.homesmartmesh.com/docs/networks/ultrawideband/

# usage
```bash
west init -m https://github.com/nRFMesh/sdk-uwb-zephyr --mr main
```

# takeover
driver zephyr port from https://github.com/foldedtoad/dwm1001.git

* `uwb\drivers\dw1000\decadriver`
* `uwb\drivers\dw1000\platform`
