#pragma once
#include <vector>

#include "http_client/http_client.h"
#include "math/vector.h"
#include "mbed.h"

namespace SimpleSlam {
class BufferedHTTPClient {
   private:
    typedef struct point_data {
        SimpleSlam::Math::Vector2 spatial_point;
        SimpleSlam::Math::Vector2 position_point;
    } point_data_t;

    SimpleSlam::HttpClient _http_client;
    size_t _capacity;
    Mutex _mutex;
    ConditionVariable _cond_var;
    std::string _host;
    std::vector<point_data_t> _buffered_data;

   public:
    BufferedHTTPClient(SimpleSlam::HttpClient& http_client, size_t capacity,
                       std::string host);
    void begin_processing();
    void add_data(point_data_t const& data);
};

}  // namespace SimpleSlam
