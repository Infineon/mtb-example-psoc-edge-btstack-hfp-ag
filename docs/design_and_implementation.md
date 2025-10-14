[Click here](../README.md) to view the README.

## Design and implementation

The design of this application is minimalistic to get started with code examples on PSOC&trade; Edge MCU devices. All PSOC&trade; Edge E84 MCU applications have a dual-CPU three-project structure to develop code for the CM33 and CM55 cores. The CM33 core has two separate projects for the secure processing environment (SPE) and non-secure processing environment (NSPE). A project folder consists of various subfolders, each denoting a specific aspect of the project. The three project folders are as follows:

**Table 1. Application projects**

Project | Description
--------|------------------------
*proj_cm33_s* | Project for CM33 secure processing environment (SPE)
*proj_cm33_ns* | Project for CM33 non-secure processing environment (NSPE)
*proj_cm55* | CM55 project

<br>

In this code example, at device reset, the secure boot process starts from the ROM boot with the secure enclave (SE) as the root of trust (RoT). From the secure enclave, the boot flow is passed on to the system CPU subsystem, where the secure CM33 application starts. After all necessary secure configurations, the flow is passed on to the non-secure CM33 application. Resource initialization for this example is performed by this CM33 non-secure project. It configures the system clocks, pins, clock to peripheral connections, and other platform resources. It then enables the CM55 core using the `Cy_SysEnableCM55()` function.

The CM33 non-secure project performs the following steps:

1. Initializes the AIROC&trade; Bluetooth&reg; stack (BTstack) for the PSOC&trade; Edge E84 MCU

2. Configures and registers the hands-free profile (HFP) audio gateway (AG) service with the Bluetooth&reg; stack

3. After connection, it initializes UART logging, GPIOs for buttons, and other peripherals required for the HFP functionality

4. Starts Bluetooth&reg; advertising and waits for a hands-free device to connect

5. Detects button actions on USER_BTN1 and USER_BTN2

6. Simulates an call when USER_BTN1 is long pressed

7. Establishes a synchronous connection-oriented (SCO) audio connection to enable two-way voice communication, where incoming voice from the HF device is played through the EVK’s speaker and outgoing voice captured by the EVK’s microphone is sent to the HF device

8. Ends the call when USER_BTN2 is long pressed

9. Terminates the SCO audio connection

10. Increases the speaker volume during an active call when USER_BTN1 is short pressed

11. Decreases the speaker volume during an active call when USER_BTN2 is short pressed

<br>