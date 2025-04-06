Currently implemented for a GQ Electronics GMC-500+ over serial USB (defaults to /dev/ttyUSB0) to:
(1) read, at serial configuration time, device model (hardware and firmware) and serial number.
(2) loop indefinitely to read CPM with specified periodicity (defaults to 30 seconds)
(3) report such CPM to stdout and to a MQTT broker (defaults to mqtt://localhost) with topic (defaults to sensors/radiation/cpm)
(4) if serial disconnection or fault, will wait and poll until serial is reconnected, then resume (at step 1)
The communication specification from GC Electronics is at https://www.gqelectronicsllc.com/download/GQ-RFC1201.txt.
Implemented in JavaScript (for NodeJS) and C. AI assisted development.
Also provided a systemctl file.
