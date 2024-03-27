#pragma once

#include "mbed.h"
#include "WiFiInterface.h"
#include "data/json.h"

#define RESPONSE_SIZE 1024
namespace SimpleSlam {

class HttpClient {
    public:
        enum class ErrorCode {
            WIFI_CONNECT_ERROR = 1,
            POST_NOT_OK = 2,
            GET_NOT_OK = 3,
            DELETE_NOT_OK = 4,
        };

    private:
        TCPSocket _socket;
        std::unique_ptr<WiFiInterface> _wifi;
        SocketAddress _addr;
        std::string error_message(ErrorCode error);

    public:
        typedef std::pair<ErrorCode, std::string> error_t;

        HttpClient(std::unique_ptr<WiFiInterface> wifi);
        HttpClient(HttpClient&& other);

        std::optional<error_t> init();

        std::optional<error_t> post_request(std::string host, std::string endpoint, JSON body_json);

        std::optional<error_t> get_request(std::string host, std::string endpoint);

        std::optional<error_t> delete_request(std::string host, std::string endpoint);
};

}
