
# Running an EPICS IOC Against the Power PMAC Simulator

This guide explains how to run an EPICS IOC (Input/Output Controller) against the Power PMAC simulator. This is useful for development and testing of EPICS support without requiring physical hardware.

## Step 1: Launch the Power PMAC Simulator

1. Run the Power PMAC Simulator.
2. Choose the desired CPU.
3. Start the simulator and confirm it is running with the default IP address `172.20.0.200`.
4. Ensure SSH access is enabled on the Windows workstation.

## Step 2: Forward the SSH Port

From your Linux development host, forward the simulator's SSH port using:

```bash
ssh -L 2222:172.20.0.200:22 $USER@<windows-workstation-hostname-or-ip>
```

Enter your password if requested.

## Step 3: Run the IOC

Configure the IOC to use the forwarded SSH port with the `pmacAsynSSHPort` class:

```tcl
# st.cmd
< envPaths
cd ${TOP}

# Use SSH port forwarding to connect to simulator
pmacAsynSSHPortConfigure("SIM_PMAC_PORT", "localhost", 2222, "root", "deltatau")

# Create the PMAC controller
pmacCreateController("SIM_PMAC", "SIM_PMAC_PORT", 0, 0)
```

Start the IOC:

```bash
./bin/linux-x86_64/pmacSim st.cmd
```

You should see output indicating a successful SSH connection to the simulator.

## Notes

- The simulator does not emulate all hardware features. Some commands may not behave identically to a real PMAC.
- SSH port forwarding allows secure and flexible access to the simulator from remote development environments.