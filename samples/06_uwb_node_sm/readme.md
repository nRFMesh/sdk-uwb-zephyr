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

nrfjprog --reset --snr 760130093
nrfjprog --reset --snr 760130128
```

- 760130093 => CBC216DC164B1DE8
- 760130128 => 1CF6567337562176
### sys commands
```shell
sm/CBC216DC164B1DE8{"sys_cmd":"reboot"}


```

### rf commands
```shell
sm/DC1119997A56350D{"rf_cmd":"ping"}
sm/1CF6567337562176{"rf_cmd":"sid"}
sm/Tag{"rf_cmd":"ping"}
sm/2{"rf_cmd":"ping"}

sm/1CF6567337562176{"rf_cmd":"target_ping","target":"CBC216DC164B1DE8"}

sm/1CF6567337562176{"rf_cmd":"target_ping","target":"CBC216DC164B1DE8"}
sm/E8D81FEE52C283EB{"rf_cmd":"ping"}

get:
sm/CBC216DC164B1DE8{"rf_cmd":"sid"}
set:
sm/CBC216DC164B1DE8{"rf_cmd":"sid","sid":1}

```

### uwb commands
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

sm{"uwb_cmd":"ping", "pinger":4,"target":1,"at_ms":100}
sm{"uwb_cmd":"twr","initiator":4,"responders":[0,1,2,3],"at_ms":100,"step_ms":10}
sm{"uwb_cmd":"twr","initiator":4,"responders":[0,1,2,3],"at_ms":100,"step_ms":10,"count":3,"count_ms":50}

sm{"uwb_cmd":"twr","initiator":0,"responders":[1],"at_ms":100,"step_ms":10}
sm{"uwb_cmd":"twr","initiator":1,"responders":[8,2,7,3,9,5,4,6],"at_ms":100,"step_ms":10}
sm{"uwb_cmd":"ping", "pinger":6,"target":5,"at_ms":100}


sm{"uwb_cmd":"ping", "pinger":2,"target":1,"at_ms":100}
resp
sm/1CF6567337562176{"uwb_cmd":"cir_acc"}

pinger 2
sm/530BE91D3559D690{"rf_cmd":"ping"}

target 1
sm/1CF6567337562176{"rf_cmd":"ping"}
```
