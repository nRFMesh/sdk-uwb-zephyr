# sdk-uwb-zephyr
Zephyr sdk for uwb module DWM1001C based on nRF52832 and DW1000 Transceiver
# Discourse forum
get support, give feedback or simply chat to brainstorm about ideas on the forum

<img src="./media/forum.png" width="400" href="https://homesmartmesh.discourse.group/c/networks/ultrawideband/10">

[Home Smart Mesh Forum - Ultra Wide Band Category](https://homesmartmesh.discourse.group/c/networks/ultrawideband/10)

# usage
```bash
west init -m https://github.com/nRFMesh/sdk-uwb-zephyr --mr main
```

# takeover
driver zephyr port from https://github.com/foldedtoad/dwm1001.git

* `uwb\drivers\dw1000\decadriver`
* `uwb\drivers\dw1000\platform`
