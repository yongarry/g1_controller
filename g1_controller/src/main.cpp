#include <iostream>
#include <string>
#include <unistd.h>

#include "g1_controller/g1_controller.h"

int main(int argc, char const *argv[]) {
    if (argc < 2) {
        std::cout << "Usage: g1_controller network_interface" << std::endl;
        return 1;
    }
    std::string networkInterface = argv[1];
    G1Controller g1_controller(networkInterface);
    while (true) sleep(10);
    return 0;
}