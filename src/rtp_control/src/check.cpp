#include <iostream>

int main() {
    // Example uint16_t value
    uint16_t uintValue =  65535;

    // Convert uint16_t to double
    int16_t doubleValue = static_cast<int16_t>(uintValue);

    // Print the results
    std::cout << "uint16_t value: " << uintValue << std::endl;
    std::cout << "Converted to double: " << doubleValue << std::endl;

    return 0;
}
