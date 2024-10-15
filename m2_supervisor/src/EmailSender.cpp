#include "EmailSender.h"
#include <iostream>
#include <cstring> // For memcpy

EmailSender::EmailSender() : curl(nullptr), messageOffset(0)
{
    curl_global_init(CURL_GLOBAL_DEFAULT);
    curl = curl_easy_init();
}

EmailSender::~EmailSender()
{
    if (curl)
    {
        curl_easy_cleanup(curl);
    }
    curl_global_cleanup();
}

void EmailSender::setCredentials(const std::string &smtpServer, const std::string &username, const std::string &password)
{
    this->smtpServer = smtpServer;
    this->username = username;
    this->password = password;
}

bool EmailSender::sendEmail(const std::vector<std::string> &recipients, const std::string &subject, const std::string &body)
{
    if (!curl)
    {
        std::cerr << "Failed to initialize libcurl." << std::endl;
        return false;
    }

    // Reset the CURL handle to ensure a clean state
    curl_easy_reset(curl);

    std::string url = "smtp://" + smtpServer;

    CURLcode resURL = curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    // std::cout << "URL: " << curl_easy_strerror(resURL) << std::endl;

    CURLcode resUsername = curl_easy_setopt(curl, CURLOPT_USERNAME, username.c_str());
    // std::cout << "Username: " << curl_easy_strerror(resUsername) << std::endl;

    CURLcode resPassword = curl_easy_setopt(curl, CURLOPT_PASSWORD, password.c_str());
    // std::cout << "Password: " << curl_easy_strerror(resPassword) << std::endl;

    CURLcode resMailFrom = curl_easy_setopt(curl, CURLOPT_MAIL_FROM, ("<" + username + ">").c_str());
    // std::cout << "Mail From: " << curl_easy_strerror(resMailFrom) << std::endl;

    struct curl_slist *recipientsList = nullptr;

    for (const auto &recipient_curl : recipients)
    {
        recipientsList = curl_slist_append(recipientsList, ("<" + recipient_curl + ">").c_str());
    }
    // std::cout << "curl_slist_append done" <<std::endl;

    CURLcode resMailRcpt = curl_easy_setopt(curl, CURLOPT_MAIL_RCPT, recipientsList);
    // std::cout << "Mail Recipient: " << curl_easy_strerror(resMailRcpt) << std::endl;

    // Simplified email body for testing
    message = "To: ";
    std::cout << message << std::endl;
    for (const auto &recipient : recipients)
    {
        std::cout << &recipient << std::endl;
        message += recipient + ",";
    }
    message.pop_back(); // Remove the trailing comma
    message += "\r\n";
    message += "From: " + username + "\r\n";
    message += "Subject: " + subject + "\r\n\r\n";
    message += body;
    messageOffset = 0;

    // Enable verbose output for debugging
    CURLcode resVerbose = curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);
    // std::cout << "Verbose: " << curl_easy_strerror(resVerbose) << std::endl;

    // Ensure SSL/TLS is used if required by your SMTP server
    CURLcode resUseSSL = curl_easy_setopt(curl, CURLOPT_USE_SSL, CURLUSESSL_ALL);
    // std::cout << "Use SSL: " << curl_easy_strerror(resUseSSL) << std::endl;

    // Set the timeout for the entire request to 2 seconds
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 2L);

    // Set the timeout for the connection phase to 2 seconds
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 2L);

    // Set the payload source callback and the data pointer
    CURLcode resReadFunc = curl_easy_setopt(curl, CURLOPT_READFUNCTION, &EmailSender::payloadSource);
    // std::cout << "Read Function: " << curl_easy_strerror(resReadFunc) << std::endl;

    CURLcode resReadData = curl_easy_setopt(curl, CURLOPT_READDATA, this);
    // std::cout << "Read Data: " << curl_easy_strerror(resReadData) << std::endl;

    CURLcode resUpload = curl_easy_setopt(curl, CURLOPT_UPLOAD, 1L);
    // std::cout << "Upload: " << curl_easy_strerror(resUpload) << std::endl;

    std::cout << "Sending email..." << std::endl;

    CURLcode res = curl_easy_perform(curl);
    curl_slist_free_all(recipientsList);

    if (res != CURLE_OK)
    {
        std::cerr << "Failed to send email: " << curl_easy_strerror(res) << std::endl;
        return false;
    }

    std::cout << "Email sent successfully." << std::endl;

    return true;
}

size_t EmailSender::payloadSource(void *ptr, size_t size, size_t nmemb, void *userp)
{
    auto *sender = static_cast<EmailSender *>(userp);
    size_t buffer_size = size * nmemb;
    size_t remaining = sender->message.size() - sender->messageOffset;

    if (remaining == 0)
    {
        return 0;
    }

    size_t copy_size = std::min(buffer_size, remaining);
    std::memcpy(ptr, sender->message.data() + sender->messageOffset, copy_size);
    sender->messageOffset += copy_size;

    return copy_size;
}