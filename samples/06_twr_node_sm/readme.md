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
sm/E2F96EB1D7A476CC{"dwt_config":{"chan":5}}

`sm/E2F96EB1D7A476CC`
```json
{
    "dwt_config":{
        "chan":4
    }
}
```
