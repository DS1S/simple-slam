#include "mbed.h"
#include "ISM43362Interface.h"

#define RESPONSE_SIZE 1024
namespace SimpleSlam {

class HttpClient {
    private:
        TCPSocket socket;
        ISM43362Interface *wifi;
        SocketAddress addr;

    public:
        // Error handling
        enum class ErrorCode {
            WIFI_CONNECT_ERROR = 1,
            POST_NOT_OK = 2,
            GET_NOT_OK = 3,
            DELETE_NOT_OK = 4,
        };
        std::string ErrorMessage(ErrorCode error) {
            switch (error) {
                case ErrorCode::WIFI_CONNECT_ERROR:
                    return "Failed to Connect to Wifi\n";
                case ErrorCode::POST_NOT_OK:
                    return "POST Failed\n";
                case ErrorCode::GET_NOT_OK:
                    return "GET Failed\n";
                case ErrorCode::DELETE_NOT_OK:
                    return "DELETE Failed\n";
            }
            return "Failed";
        }

        typedef std::pair<ErrorCode, std::string> error_t;

        HttpClient(ISM43362Interface* wifi);

        std::optional<error_t> Init();

        std::optional<error_t> Post(std::string host, std::string endpoint, std::string body, int size);

        std::optional<error_t> Get(std::string host, std::string endpoint);

        std::optional<error_t> Delete(std::string host, std::string endpoint);
};

}
