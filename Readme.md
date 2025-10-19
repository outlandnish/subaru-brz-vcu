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
  - The certification ECU activates the ACCD line. The ACC1 and ACC2 relays are activated. ACC1 relay provides 10A of power to ECU ACC to the main body ECU.
  - The ECM activates EFI Main 1, Main 2, and Main 3 relays via SSHUT - `C13` and activates the +B switched power to the ECM (`A1` and `B6`). throttle body relay (`C17`), injectors relay (`C5`), starter relay (`C26`) are now controllable. 
  - Canister pump module is powered on
  - The certification ECU activates IG1D and IG2D. IG2D activates the IG1 No. 3 Relay (for A/C) and activates the IG1 No. 4 Relay for Seat Heaters. IG1D activates the IG2 Relay which controls power to ECU IG2, the gauge cluster, IG2, and A/B Main off of the 30A IG2 Main fuse. The IG2 Relay also activates the IGS relay (which is one half of the ST CUT relay)
2. When the starter switch is pressed to "start the engine", the following occurs:
  - The certification ECU checks the clutch switch to ensure it is not pressed. The clutch switch also activates one side of the ST Relay.
  - The certification ECU checks to ensure the key is present and sends a challenge via the IMO line to the main ECU. The ECU responds on this line back to the certification ECU (need to investigate).
  - If everything is good, the certification ECU sends the STSW signal to the ECM on `A17`
  - The ECM activates the ACCR line to request accessory power cutoff using pin `C32`. 
  - The ECM sends out the STA signal on pin `C26`
  - When the ECM sees the engine RPM (in an ICE car), it sends out a tachometer (12V PWM) on `C15` to the certification ECM and also sends out the STAR signal on `C34` to activate the ST CUT relay. (need to investigate - does this signal come from the certification ECU originally when tachometer signal gets sent to it?). The engine turns off the STA signal on `C26`
  - The ECM deactivates the ACCR line
3. When the starter switch is pressed to the off position, the following occurs:
  - The certification ECU deactivates IG1D and IG2D and deactivates the ACCR (and STSW?) signals

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

## Required I/O on the ECM
### Outputs
- Low side relay control for SSHUT, HB, FAN1, FAN2, STA, STAR, MCR, AC, ACCR
- VCPA, VCP2 (+5V) for accelerator pedal
- EPA, EPA2 (GND) for accelerator pedal
- VC (+5V) for any engine connected +5V sensors
- PWM (+5V) (TACH) for tachometer to certification ECU

The following are repurposes signals whose ground path is controlled by low-side relays
- Negative Contactor (+12V) control (IGT1)
- Pre-Charge Contactor (+12V) control (IGT2)
- Positive Contactor (+12V) control (IGT3)

### Inputs
- ADC for resistor network on Cruise Control Switch
- ADC for VPA, VPA2 (accelerator pedal signals)
- 12V input AC high side pressure switch (ACP), clutch switch (CSW), stop light switch (ST1-, A3 - normally closed, A7 - normally open), ignition switch (IGSW), neutral switch (NSW), starter switch (STSW2)
- 5V to 3.3V logic level converter for IMO (in case IMO is a digital signal instead of a 0/1)
- ADC 0.8V-1.8V ADC input for E.F.I Water Temperature Sensor

## Signals
- LIN bus (TJA1020) for A21 (battery sensor comms)
- Secondary CAN bus pins for direct communication to the Tesla DU
- Differential signals for TPL connections (BMS)

## Component replacements
- SSHUT activates EFI Main3 relay (check current ratings) which gives us control over IGT1, IGT2, IGT3, IGT4 - we can utilize these to control the contactors using an optocoupler. This way we can avoid assigning existing pins 
- Use the existing Engine Water Temperature Sensor and/or Engine Oil Temperature Sensor to sense the temp of the battery blocks (if needed)
- Use DI or FPC to control brake switch

## Notes for the inverter controller
- C/OPEN (for the fuel pump) will provide our switched +12V
- Maybe use DI and FPC cables as our secondary CAN bus to control the Ingenext controller

## Notes about power
- EC, SLE2, EC3, EC4, EC05, EKNK, SLE1, SLE3, E02, E01 are all chassis grounds + shielded wires (SLE x)
- E1 is GND for 5V circuits and connects to external pin (separately from E01) to provide shielding to certain sensors (sensor GND)
- E01 is GND for 12V circuits (power GND)
- E02 is also GND (tbd why it's used)
- ETCS +BM rail provides an additional 15A power source (previously used for the electronic throttle control). A new ground path is needed in the ECM
- STSW2 provides a 7.5A power source for the ST relay
- Permanent 12V +B (A2) is powered by a 7.5A power source (EFI +B)
- EFI Main1 provides 15A to Purge VSV (controlled by PRG), #1, #2, #3, #4 fuel injectors, EV2+, EV1+, VV2+, VV1+ VVT sensors. They all have a sensor power provided by the VCV pin of the ECM. VCV power looks to be provided by the 15V ETCS relay mentioned above
- Stop Light Switch Assembly + Clutch Switch Assembly route conditionally switched 10A power from ECU IG2 (most likely to also power brake lights)
- EFI Main2 provides 15A of power from EFI(HTR) to the heated O2 sensors. This could be another option for sensor power sources in the engine bay.
- VC pin provides +5V power to Pressure Sensor, Throttle Body w/ Motor Assembly, Crankshaft Position Sensor, and Fuel Pressure Sensor. Option for +5V in the engine bay

- A permanent 12V source is needed for the Ingenext controller
- Check to ensure that C/OPEN can provide enough switched current to the Ingenext controller

## To Do:
- investigate where the Canister Pump Module is + the VPMP, MPMP, PPMP controls. +B for the canister pump module is controlled the +B and +B2 rails of the ECM (could be an option for the Ingenext controller)
- figure out whst to do with the Ingenext relays

## Components required
- 3x CAN transcievers (SN65HVD230 or similar)
- 1x LIN transciever (SN65HVDA195 or TJA1020)
- 1x TPL transciever (MC33664)
- 1x ST L9800 for low-side relay control (BRZ electronics + pre-charge relay), 1x ST L9026 for high + low side relay control (or use two L9026 in daisy chain mode)
- 1x DRV8703-Q1 for HV contactor control

## I/O pins required
- 3x CAN (6 pins)
- 2x SPI for TPL (8 I/O), 1x SPI for 2x L9800 + 1x DRV8703 (3 pins SPI + 3 CS) (14 pins)
- 1x USART for LIN (2 pins)
- 1 pins ADC for cruise control, 2 pins analog for accelerator pedal, 1 pin for 5V ref, 1 pin for water temp sensor, 1 pin for oil temp sensor (6 pins)
- 6 pins 12V digital I/O (6 pins)
- 1 pin for for logic level translator for IMO (1 pin)
- 1 pin for IGSW and 1 pin for STSW (2 pins)
- 1 pin for PWM signal for tachometer (1 pin)
- 1 pin for enabling VC output (1 pin) (enable pin of a 5V regulator?)
- 1 pin for HV interlock?
- 2 pins for NO / NC brake switch to DI / FPC for Tesla inverter
- 3 pins optional I/O for VPMP, MPMP, and PPMP? (Tesla inverter accelerator pedal signals most likely)

## Ingenext controller requirements
- Permanent 12V
- Switched 12V
- Ground
- 5V + Signal Ground for Accelerator Pedals, Signals for Accelerator Pedals
- NO brake switch and NC brake switch
- CANH / CANL
- HVIL output / return

## Available pins in the rear of the car
Fuel Pump Control ECU Assembly, No. 2 Fuel Sender Gage Assembly, Fuel Sender Gage Assembly, and Canister Pump Module can all be utilized.

- ECM DI and FPC to Fuel Pump Control ECU Assembly (G12). Includes +B controlled by C/OPEN Relay and a chassis ground (G1). Has output pins FP- and FP+ that connect to the Fuel Pump (G7)
- Fuel Pump FS- from Combination Meter, FS- to Fuel Sender Gage Assembly, FS+ from Fuel Sender Gage Assembly to No. 2 Fuel Sender Gage Assembly with L output to Combination Meter
- ECM VPMP, MPMP, and PPMP to Canister Pump Module. +B from ETCS Relay controlled by ECM +BM line

## Accelerator Pedal Notes
- Need 0-3.3V to 0-5V Op-Amp with 1.5x gain
