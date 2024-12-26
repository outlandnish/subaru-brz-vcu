# BRZ VCU Firmware

## Description

This is a replacement ECU for a Subaru BRZ for an EV conversion. The goal of this ECU is to be a drop in place replacement that utilizes as much of the existing wiring harnesses as possible. This ECU also interfaces with the stock dashboard, body ECU, certification ECU, traction control, electric power steering, and ABS ECUs from the Subaru BRZ.

This conversion uses the following EV components:

- Tesla Model 3 Rear Drive Unit with an Ingenext Controller
- Tesla Model 3 Power Conversion System (PCS) for charging + DCDC conversion
- Dilithium distributed BMS
- IVT-S current + voltage sensor
- (future) Foccci CCS Charger
- (potential) Bosch iBooster?

## Theory of Operation

### Power

12V battery (or 12V from the PCS) provides power for lower power systems. The battery has a sensor that connects to the ECM via a LIN connection on `A21`. This LIN connection is also normally connected to the the starter on an ICE car.
Permanent 12V is provided to the ECM via `A2`. The 12V is stepped down to both 5V and 3.3V onboard for different sensors.

1. When the starter switch is pressed for accessory power, the following occurs:
  - The certification ECU activates the ACCD line. The ACC1 and ACC2 relays are activated. ACC1 relay provides 10A of power to ECU ACC to the main body ECU and (maybe?) activates the ACCR line to the main ECM on pin `C32`. 
  - The ECM activates EFI Main 1, Main 2, and Main 3 relays via SSHUT - `C13` and activates the +B switched power to the ECM (`A1` and `B6`). throttle body relay (`C17`), injectors relay (`C5`), starter relay (`C26`) are now controllable. 
  - Canister pump module is powered on
2. When the starter switch is pressed to "start the engine", the following occurs:
  - The certification ECU activates IG1D and IG2D. IG2D activates the IG1 No. 3 Relay (for A/C) and activates the IG1 No. 4 Relay for Seat Heaters. IG1D activates the IG2 Relay which controls power to ECU IG2, the gauge cluster, IG2, and A/B Main off of the 30A IG2 Main fuse. The IG2 Relay also activates the IGS relay (which is one half of the ST CUT relay)
  - The certification ECU checks the clutch switch to ensure it is active. The clutch switch also activates one side of the ST Relay.
  - The certification ECU checks to ensure the key is present and does something with the IMO line that connects to the main ECU. If this check fails, something else happens on the IMO line (need to investigate).
  - If everything is good, the certification ECU sends the STSW signal to the ECM on `A17`
  - The ECM sends out the STA signal on pin `C26`
  - When the ECM sees the engine RPM (in an ICE car), it sends out a tachometer (solid or PWM?) on `C15` to the certification ECM and also sends out the STAR signal on `C34` to activate the ST CUT relay. (need to investigate - does this signal come from the certification ECU originally when tachometer signal gets sent to it?). The engine turns off the STA signal on `C26`
3. When the starter switch is pressed to the off position, the following occurs:
  - The certification ECU deactivates IG1D and IG2D and deactivates the ACCR (and STSW?) signals

#### Changes in EV mode
1. Accessory power
  - Not sure. Mark accessory power on from signal on `C32`. Activate MAIN1, MAIN2,and MAIN3 relays

2. Ignition signal
  - 

#### HV Startup
1. Check if battery maintenance switch is closed. If not, throw a fault.
2. Check battery status. If the state of charge is too low or too high, add fault flags for battery
3. Start battery temperature regulation. If the battery modules are too cold, warm them up. If the battery modules are too cold, cool them down. Add battery temperature fault flags if necessary.
4. Check battery humidity? (TODO: decide if this is necessary)
5. If there are no battery faults, start the pre-charge process. Close negative contactor and pre-charge resistor contactor.
6. Once voltage from the voltage sensor roughly matches the battery voltage, close the positive connector and open the pre-charge resistor contactor.
7. Mark HV status as ready

##### Notes
In order for the car to charge it's 12V battery or to startup the heating / cooling process, the contactors need to be controllable regardless of ignition state. Look for which power sources are active (and controllable) in such state

#### Low Voltage Power Management
1. Check 12V battery state from LIN. If the battery is too low or there is too much current being drawn, start PCS (shown below). Mark 12V battery state as low?
1. Check HV status. If it is disconnected, run HV startup process
2. When HV status is ready, start PCS charging (look into firmware that Damien McGuire wrote for his stuff). Mark 12V battery state as charging if command succeeds
3. When the 12V sensor is in a healthy state, stop PCS charging. If HV status falls out of connected / healthy state, stop PCS charging
4. Update 12V battery state

#### Questions

- What needs to turn on with switched +12V power? We can activate that from the ACCR signal and re-utilize the power lines to the canister module as well.
