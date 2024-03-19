#include "mbed.h"
#include "WiFiInterface.h"

#define RESPONSE_SIZE 1024
namespace SimpleSlam {

class HttpClient {
    private:
        TCPSocket _socket;
        std::unique_ptr<WiFiInterface> _wifi;
        SocketAddress _addr;

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

        HttpClient(WiFiInterface* wifi);

        std::optional<error_t> init();

        std::optional<error_t> deinit();

        std::optional<error_t> post(std::string host, std::string endpoint, std::string body);

        std::optional<error_t> get(std::string host, std::string endpoint);

        std::optional<error_t> delete_request(std::string host, std::string endpoint);
};

}
