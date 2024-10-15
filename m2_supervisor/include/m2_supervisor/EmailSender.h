#ifndef EMAILSENDER_H
#define EMAILSENDER_H

#include <string>
#include <vector>
#include <curl/curl.h>

class EmailSender {
public:
    EmailSender();
    ~EmailSender();

    void setCredentials(const std::string &smtpServer, const std::string &username, const std::string &password);
    bool sendEmail(const std::vector<std::string> &recipients, const std::string &subject, const std::string &body);

private:
    CURL *curl;
    std::string smtpServer;
    std::string username;
    std::string password;
    std::string message;
    size_t messageOffset;

    static size_t payloadSource(void *ptr, size_t size, size_t nmemb, void *userp);
};

#endif // EMAILSENDER_H
