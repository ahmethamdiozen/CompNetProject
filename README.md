
# NS3 LoRaWAN Simulation

## Description
This project contains a simulation scenario for **LoRaWAN** communication using the **NS-3** simulator. In the simulation, packet transmission between LoRaWAN devices is simulated, and metrics such as the number of sent and received packets, packet delivery rates, and delays are tracked.

## Features
- Packet transmission between **LoRaWAN** devices and gateways.
- Support for different interference matrices such as **ALOHA** and **Goursaud**.
- **NS-2** mobility trace file used for mobile devices.
- Calculation of sent and received packet counts and packet delivery rate.
- Calculation of packet delays.

## Dependencies
- **NS-3** (Version 3.41 or newer)
- LoRaWAN and related modules
- **Tcl** file: `vanetmobility.tcl` (NS-2 mobility trace data)

## Installation
1. **Install NS-3**:
   - Follow the [NS-3 installation guide](https://www.nsnam.org/wiki/Installation).

2. **Download the project files**:
   - Place this file in the appropriate location within your NS-3 directory.

3. **Obtain the Tcl trace file**:
   - Place the `vanetmobility.tcl` file in the directory: `/home/bilmuh/ns-allinone-3.41/ns-3.41/src/lorawan/examples/`.
   - You can use a different NS-2 trace file, but this file is specified as the default in the project.

## Command-Line Arguments
You can use the following command-line arguments when running this file:

```
--numDevices      : Specifies the number of devices to include in the simulation (default: 50)
--simTime         : Simulation time in seconds (default: 50 seconds)
--interferenceMatrix : Select the interference matrix [aloha, goursaud] (default: aloha)
--traceFile       : Path to the NS2 mobility trace file (default: vanetmobility.tcl)
```

Example command:
```bash
./lora_simulation --numDevices=100 --simTime=60 --interferenceMatrix=aloha --traceFile=path/to/traceFile.tcl
```

## Output
When the simulation runs, the following results will be printed to the console:

- Number of sent and received packets.
- Packet delivery rate (%).
- Average packet delay (in seconds).

Example output:
```
Packet Sent: 200
Packet Received: 180
Packet Delivery Rate: 90%
Average delay: 1.25 seconds
```

## Simulation Setup
- **Network Setup**: `numDevices` devices and `numGateways` gateways are created.
- **Mobility Model**: Devices are moved based on an **ns-2** mobility trace file.
- **Physical Layer and MAC**: LoRa PHY and LoRaWAN MAC layers are configured for both devices and gateways.
- **Packet Transmission**: A periodic packet transmission application is started for each device.

## Contribution
Feel free to contribute to the project by submitting a PR (Pull Request) or resolving any open issues.

## License
This project is licensed under the **NS-3** simulator license.
