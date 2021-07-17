# Listener Simple Mesh
## usage
```bash
west build -b decawave_dwm1001_dev -- -DCONF_FILE=prj.conf
west build -b decawave_dwm1001_dev -- -DCONF_FILE="prj.conf overlay-debug.conf"
west build -b decawave_dwm1001_dev -- -DCONF_FILE="prj.conf overlay-tracing.conf"
west build -b decawave_dwm1001_dev -- -DCONF_FILE="prj.conf overlay-usb.conf"

west build -t guiconfig -b decawave_dwm1001_dev

west flash

west flash --snr 760130093
west flash --snr 760130128
west flash --snr 760130140

nrfjprog --reset --snr 760130093
nrfjprog --reset --snr 760130128
```

- 760130093 => CBC216DC164B1DE8
- 760130128 => 1CF6567337562176
### commands
```shell
get:
sm/1CF6567337562176{"uwb_cmd":"config"}
sm{"uwb_cmd":"config","chan":5}

set:
sm{"uwb_cmd":"config"}
sm/1CF6567337562176{"uwb_cmd":"config","chan":5}

sm/1CF6567337562176{"rf_cmd":"ping"}
sm/1CF6567337562176{"rf_cmd":"target_ping","target":"CBC216DC164B1DE8"}
sm/E8D81FEE52C283EB{"rf_cmd":"ping"}

sm{"uwb_cmd":"twr","initiator":0,"responder":1,"at_ms":200}
sm{"uwb_cmd":"twr","initiator":0,"responder":3,"at_ms":100}

sm{"uwb_cmd":"twr","initiator":0,"responder":1,"at_ms":100,"count":3,"count_ms":20}

sm{"uwb_cmd":"twr","initiator":0,"responders":[1,1],"at_ms":100,"step_ms":10}
sm{"uwb_cmd":"twr","initiator":0,"responders":[1,2],"at_ms":100,"step_ms":10}
sm{"uwb_cmd":"twr","initiator":0,"responders":[1,1],"at_ms":100,"step_ms":10,"count":3,"count_ms":50}


sm{"uwb_cmd":"ping", "pinger":0,"target":1,"at_ms":100}
sm{"uwb_cmd":"ping", "pinger":0,"target":1,"at_ms":100,"count":3,"count_ms":6}

sm/90A971A3D1A1B648{"rf_cmd":"target_ping","target":"98501ED22B42EB41"}
sm{"uwb_cmd":"ping", "pinger":4,"target":1,"at_ms":100}
sm{"uwb_cmd":"twr","initiator":4,"responders":[0,1,2,3],"at_ms":100,"step_ms":10}
sm{"uwb_cmd":"twr","initiator":4,"responders":[0,1,2,3],"at_ms":100,"step_ms":10,"count":3,"count_ms":50}

```

