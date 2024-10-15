#include "Snoozer.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <unordered_map>

Snoozer::Snoozer() {}

bool Snoozer::snoozeInput(const std::string &tagName, int snoozeTime)
{
    auto currentTime = std::chrono::steady_clock::now();

    // Check if tagName exists in the map
    if (tagTimes.find(tagName) != tagTimes.end())
    {
        // Check if snoozeTime seconds have passed since the last time tagName was received
        auto elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(currentTime - tagTimes[tagName]);
        if (elapsedTime.count() < snoozeTime)
        {
            return true; // tagName received within snoozeTime seconds
        }
    }

    // Update the time for the tagName
    tagTimes[tagName] = currentTime;
    return false; // tagName not received within snoozeTime seconds
}
