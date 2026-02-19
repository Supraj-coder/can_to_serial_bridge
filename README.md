# Setup
1. Connect the system with the serial adapter
2. Check the serial port (e.g /dev/ttyACM0)
4. Build the file

   
   ```bash
   g++ bridge_bi.cpp -o bridge -lpthread
   ```

   
6. Setup the virtual can interface
   ```bash
   sudo modprobe vcan
   sudo ip link add dev can0 type vcan
   sudo ip link set can0 txqueuelen 1000
   sudo ip link set up can0
   ```
7. Run the bridge
   ```bash
   ./bridge /dev/ttyACM0
   ```
## Verifiaction
1. Terminal 1
   ```bash
   candump can0
   ```
2. Terminal 2
   ```bash
   cansend can0 010#FFFFFFFFFFFFFFFC
   ```
   The message should reflect in the Terminal 1
