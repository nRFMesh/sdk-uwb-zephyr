# Listener Simple Mesh
## usage
```bash
west build -t guiconfig -b decawave_dwm1001_dev

west build -b decawave_dwm1001_dev -- -DCONF_FILE=prj.conf
west build -b decawave_dwm1001_dev -- -DCONF_FILE="prj.conf overlay-debug.conf"
west build -b decawave_dwm1001_dev -- -DCONF_FILE="prj.conf overlay-tracing.conf"
west build -b decawave_dwm1001_dev -- -DCONF_FILE="prj.conf overlay-usb.conf"

west flash

west flash --snr 760130093
west flash --snr 760130128

nrfjprog --reset --snr 760130093
nrfjprog --reset --snr 760130128
```

## uwb json config
- 760130093 => CBC216DC164B1DE8
- 760130128 => 1CF6567337562176
request to node with uid
`sm/1CF6567337562176{"dwt_config":{"chan":5}}`
broadcast a channel for all
`sm{"dwt_config":{"chan":5}}`

broadcast to all nodes
`sm{"twr_command":{"initiator":0,"responder":1,"at_ms":50}}`
`sm{"twr_command":{"initiator":0,"responder":1,"at_ms":100}}`
`sm{"twr_command":{"initiator":1,"responder":0}}`

response on topic : `sm/E2F96EB1D7A476CC`
```json
{
    "chan":5,
    "dataRate":"DWT_BR_6M8",
    "nsSFD":"NonStandard",
    "phrMode":"DWT_PHRMODE_EXT",
    "prf":"DWT_PRF_64M",
    "rxCode":9,
    "rxPAC":"DWT_PAC8",
    "sfdTO":129,
    "txCode":9,
    "txPreambLength":"DWT_PLEN_128"
}
```

