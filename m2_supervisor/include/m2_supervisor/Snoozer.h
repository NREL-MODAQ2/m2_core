#ifndef SNOOZER_H
#define SNOOZER_H

#include <string>
#include <iostream>
#include <thread>
#include <unordered_map>


class Snoozer {
public:
    Snoozer();

    // Function to check if tagName has been received before snoozeTime seconds
    bool snoozeInput(const std::string& tagName, int snoozeTime);

private:
    std::unordered_map<std::string, std::chrono::steady_clock::time_point> tagTimes;
};

#endif // SNOOZER_H
