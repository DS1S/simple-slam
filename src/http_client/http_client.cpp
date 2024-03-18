#include <utility>
#include <string>
#include <optional>
#include "ISM43362Interface.h"
#include "mbed.h"
#include "http_client/http_client.h"
#include "http_client/wifi_config.h"

using namespace SimpleSlam;

SimpleSlam::HttpClient::HttpClient(ISM43362Interface* wifi) : wifi(wifi) {}

std::optional<HttpClient::error_t> HttpClient::Http_Client_Init() {
    printf("[HttpClient]: Http Client Init\n");
    nsapi_error_t error = wifi->connect(WIFI_SSID, WIFI_PASS, NSAPI_SECURITY_WPA2);
    if (error != 0) {
        return std::make_optional(std::make_pair(    \
            ErrorCode::WIFI_CONNECT_ERROR,           \
            "Failed to Connect to Wifi"));
    }
    return {};
}

std::optional<HttpClient::error_t> HttpClient::Post(std::string host, std::string endpoint, std::string body, int size) {
    string request;
    request.append("POST ").append(endpoint).append(" HTTP/1.1\r\n")
        .append("Host: ").append(host).append("\r\n")
        .append("Content-Type: ").append("application/json").append("\r\n")
        .append("Content-Length: ").append(std::to_string(size)).append("\r\n\r\n")
        .append(body);

    ((NetworkStack *) wifi)->gethostbyname(host.c_str(), &addr);
    addr.set_port(80);
    socket.open(wifi);
    socket.connect(addr);
    socket.send(request.c_str(), request.length());

    char buffer[16];
    socket.recv(buffer, 16);
    if (strncmp(buffer, "HTTP/1.1 200 OK", 15) != 0) {
        printf("POST Response: %s\n", buffer);
        return std::make_optional(std::make_pair(
            ErrorCode::POST_NOT_OK,
            "POST Failed\n"));
    }
    socket.close();
    return {};
}

std::optional<HttpClient::error_t> HttpClient::Get(std::string host, std::string endpoint, char* buffer) {
    string request;
    request.append("GET ").append(endpoint).append(" HTTP/1.1\r\n")
        .append("Host: ").append(host).append("\r\n\r\n");

    ((NetworkStack *) wifi)->gethostbyname(host.c_str(), &addr);
    addr.set_port(80);
    socket.open(wifi);
    socket.connect(addr);
    socket.send(request.c_str(), request.length());

    memset(buffer, 0, RESPONSE_SIZE);
    socket.recv(buffer, RESPONSE_SIZE);
    if (strncmp(buffer, "HTTP/1.1 200 OK", 15)) {
        return std::make_optional(std::make_pair(
            ErrorCode::GET_NOT_OK,
            "GET Failed\n"));
    }
    socket.close();
    return {};
}
