# Listener Simple Mesh
## usage
```bash
west build -t guiconfig -b decawave_dwm1001_dev

west build -b decawave_dwm1001_dev -- -DCONF_FILE=prj.conf
west build -b decawave_dwm1001_dev -- -DCONF_FILE="prj.conf overlay-debug.conf"
west build -b decawave_dwm1001_dev -- -DCONF_FILE="prj.conf overlay-tracing.conf"
west build -b decawave_dwm1001_dev -- -DCONF_FILE="prj.conf overlay-usb.conf"

west flash

west flash --snr 760130125
```

## uwb json config
request
`sm/E2F96EB1D7A476CC{"dwt_config":{"chan":5}}`

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

