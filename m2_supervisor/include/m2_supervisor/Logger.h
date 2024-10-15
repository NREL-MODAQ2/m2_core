// Logger.h
#ifndef LOGGER_H
#define LOGGER_H

#include <string>
#include <iostream>
#include <fstream>

class Logger {
public:
    Logger();
    ~Logger();

    void log(const std::string& data);
    void setBasePathandSize(const std::string& basePath, const int byteLimit);

private:
    std::string generateFilename();
    void checkFileSize();

private:
    std::string currentFilename;
    std::ofstream outputFile;
    std::streampos fileSizeLimit;
    std::string basePath;
};

#endif // LOGGER_H
