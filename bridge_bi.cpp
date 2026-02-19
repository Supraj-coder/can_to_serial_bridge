#include <iostream>
#include <vector>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <thread>
#include <map>
#include <mutex>

// --- Latching Logic ---
struct MotorCommand {
    uint8_t data[30];
    bool active = false;
};

std::map<uint32_t, MotorCommand> latches;
std::mutex latch_mtx;

// Repeats the last sent command for every motor at 50Hz
void command_repeater_thread(int ser_fd) {
    while (true) {
        {
            std::lock_guard<std::mutex> lock(latch_mtx);
            for (auto it = latches.begin(); it != latches.end(); ++it) {
                if (it->second.active) {
                    write(ser_fd, it->second.data, 30);
                }
            }
        }
        usleep(20000); // 20ms = 50Hz
    }
}

// Thread to handle Motor -> PC (Feedback)
void serial_to_can_thread(int ser_fd, int can_sock) {
    uint8_t buffer[64];
    struct can_frame frame;
    while (true) {
        uint8_t start_byte;
        if (read(ser_fd, &start_byte, 1) <= 0) continue;

        if (start_byte == 0xAA) {
            int received = 1;
            while (received < 16) {
                int n = read(ser_fd, buffer + received, 16 - received);
                if (n > 0) received += n;
            }
            std::memset(&frame, 0, sizeof(frame));
            frame.can_id = buffer[7];
            frame.can_dlc = 8;
            std::memcpy(frame.data, &buffer[8], 8);
            write(can_sock, &frame, sizeof(struct can_frame));
        } 
        else if (start_byte == 0x55) {
            int received = 1;
            while (received < 30) {
                int n = read(ser_fd, buffer + received, 30 - received);
                if (n > 0) received += n;
            }
        }
    }
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <serial_port>" << std::endl;
        return 1;
    }

    int can_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, "can0"); 
    ioctl(can_sock, SIOCGIFINDEX, &ifr);
    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(can_sock, (struct sockaddr *)&addr, sizeof(addr));

    int ser_fd = open(argv[1], O_RDWR | O_NOCTTY | O_SYNC);
    if (ser_fd < 0) { perror("Serial Port Error"); return 1; }

    struct termios tty;
    tcgetattr(ser_fd, &tty);
    cfsetospeed(&tty, B921600);
    cfsetispeed(&tty, B921600);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8 | CLOCAL | CREAD;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);
    tty.c_lflag = 0; tty.c_oflag = 0; 
    tcsetattr(ser_fd, TCSANOW, &tty);

    std::cout << "Latching Bridge Active on " << argv[1] << std::endl;

    std::thread feedback_thread(serial_to_can_thread, ser_fd, can_sock);
    std::thread repeater_thread(command_repeater_thread, ser_fd);
    feedback_thread.detach();
    repeater_thread.detach();

    struct can_frame frame;
    uint8_t dm_packet[30];

    while (true) {
        if (read(can_sock, &frame, sizeof(struct can_frame)) > 0) {
            std::memset(dm_packet, 0, 30);
            dm_packet[0] = 0x55; 
            dm_packet[1] = 0xAA;
            dm_packet[2] = 0x1E;
            dm_packet[3] = 0x03; 
            dm_packet[4] = 0x01;
            dm_packet[8] = 0x0A; 
            dm_packet[13] = frame.can_id & 0xFF; 
            dm_packet[14] = (frame.can_id >> 8) & 0xFF;
            dm_packet[18] = 0x08; 

            uint8_t last_byte = frame.data[7];
            if (last_byte >= 0xFB && last_byte <= 0xFD) {
                for (int i = 0; i < 7; i++) dm_packet[21 + i] = 0xFF;
                dm_packet[28] = last_byte;
            } else {
                for (int i = 0; i < 8; i++) dm_packet[21 + i] = frame.data[i];
            }

            // Update Latch
            {
                std::lock_guard<std::mutex> lock(latch_mtx);
                std::memcpy(latches[frame.can_id].data, dm_packet, 30);
                latches[frame.can_id].active = true;
            }
            write(ser_fd, dm_packet, 30);
        }
    }
    return 0;
}
