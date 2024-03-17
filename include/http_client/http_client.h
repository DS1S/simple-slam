#include <ISM43362Interface.h>

#define RESPONSE_SIZE 1024
namespace SimpleSlam::HttpClient {

// Error handling
enum class ErrorCode {
    WIFI_CONNECT_ERROR = 1,
    POST_NOT_OK = 2,
    GET_NOT_OK = 3,
};

typedef std::pair<ErrorCode, std::string> error_t;

std::optional<error_t> Http_Client_Init();

std::optional<error_t> Post(std::string host, std::string endpoint, std::string body, int size);

std::optional<error_t> Get(std::string host, std::string endpoint, char* buffer);

}
