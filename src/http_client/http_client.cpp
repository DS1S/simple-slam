#include <utility>
#include <string>
#include <optional>
#include "WiFiInterface.h"
#include "mbed.h"
#include "http_client/http_client.h"
#include "http_client/wifi_config.h"
#include "data/header.h"

using namespace SimpleSlam;

SimpleSlam::HttpClient::HttpClient(WiFiInterface* wifi) : _wifi(wifi) {}

std::optional<HttpClient::error_t> HttpClient::init() {
    printf("[HttpClient]: Http Client Init\n");
    nsapi_error_t error = _wifi->connect(WIFI_SSID, WIFI_PASS, NSAPI_SECURITY_WPA2);
    if (error != 0) {
        return std::make_optional(std::make_pair(    \
            ErrorCode::WIFI_CONNECT_ERROR,           \
            ErrorMessage(ErrorCode::WIFI_CONNECT_ERROR)));
    }
    return {};
}

std::optional<HttpClient::error_t> HttpClient::deinit() {
    _wifi.release();
    return {};
}

std::optional<HttpClient::error_t> HttpClient::post(std::string host, std::string endpoint, std::string body) {
    SimpleSlam::Header header;
    string request;
    header
        .request_type(SimpleSlam::HTTPRequestType::POST, endpoint)
        .add("Host", host)
        .add("Content-Type", "application/json")
        .add("Content-Length", std::to_string(body.length()));

    std::string header_str = header.build();
    request
        .append(header_str)
        .append("\r\n")
        .append(body);

    _wifi->gethostbyname(host.c_str(), &_addr);
    _addr.set_port(80);
    _socket.open(_wifi.get());
    _socket.connect(_addr);
    _socket.send(request.c_str(), request.length());

    char buffer[16];
    _socket.recv(buffer, 16);
    if (strncmp(buffer, "HTTP/1.1 200 OK", 15) != 0) {
        printf("POST Response: %s\n", buffer);
        return std::make_optional(std::make_pair(
            ErrorCode::POST_NOT_OK,
            ErrorMessage(ErrorCode::POST_NOT_OK)));
    }
    _socket.close();
    return {};
}

std::optional<HttpClient::error_t> HttpClient::get(std::string host, std::string endpoint) {
    SimpleSlam::Header header;
    string request;
    header
        .request_type(SimpleSlam::HTTPRequestType::GET, endpoint);
        .add("Host", host);

    std::string header_str = header.build();
    request
        .append(header_str)
        .append("\r\n");

    _wifi->gethostbyname(host.c_str(), &_addr);
    _addr.set_port(80);
    _socket.open(_wifi.get());
    _socket.connect(_addr);
    _socket.send(request.c_str(), request.length());

    char buffer[16];
    _socket.recv(buffer, 16);
    if (strncmp(buffer, "HTTP/1.1 200 OK", 15) != 0) {
        printf("GET Response: %s\n", buffer);
        return std::make_optional(std::make_pair(
            ErrorCode::GET_NOT_OK,
            ErrorMessage(ErrorCode::GET_NOT_OK)));
    }
    _socket.close();
    return {};
}

std::optional<HttpClient::error_t> HttpClient::delete_request(std::string host, std::string endpoint) {
    SimpleSlam::Header header;
    string request;
    header
        .request_type(SimpleSlam::HTTPRequestType::DELETE, endpoint);
        .add("Host", host);

    std::string header_str = header.build();
    request
        .append(header_str)
        .append("\r\n");

    _wifi->gethostbyname(host.c_str(), &_addr);
    _addr.set_port(80);
    _socket.open(_wifi.get());
    _socket.connect(_addr);
    _socket.send(request.c_str(), request.length());

    char buffer[16];
    _socket.recv(buffer, 16);
    if (strncmp(buffer, "HTTP/1.1 200 OK", 15) != 0) {
        printf("DELETE Response: %s\n", buffer);
        return std::make_optional(std::make_pair(
            ErrorCode::DELETE_NOT_OK,
            ErrorMessage(ErrorCode::DELETE_NOT_OK)));
    }
    _socket.close();
    return {};
}
