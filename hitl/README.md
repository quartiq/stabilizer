# Stabilizer HITL Testing

This directory contains tooling required for Stabilizer hardware-in-the-loop (HITL) testing.

There is a `Stabilizer` board connected at the Quartiq office that is accessible via a private HITL
repository in order to provide secure hardware testing of the stabilizer application in a public
repository.

**Note**: In order to ensure application security, all HITL runs must first be approved by a Quartiq
representative before execution.

# Hardware Configuration
Stabilizer is configured with an ethernet connection to a router. The router runs a DHCP server for
the local network, and Stabilizers MAC address has been assigned to the unique host name of
`gonnigan.ber.quartiq.de`.

# HITL Workflow
The private HITL repository does the following:

1. Check out this repository
2. Build firmware images using Cargo
3. Program stabilizer
4. Execute `hitl/run.sh`

In order to add new HITL tests, update `run.sh` to include the necessary tests.
