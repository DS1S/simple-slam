#include "http_client/buffered_http_client.h"

SimpleSlam::BufferedHTTPClient::BufferedHTTPClient(
    SimpleSlam::HttpClient& http_client, size_t capacity, std::string host)
    : _http_client(std::move(http_client)),
      _capacity(capacity),
      _mutex(),
      _cond_var(_mutex),
      _host(std::move(host)) {
    std::optional<HttpClient::error_t> maybe_error = _http_client.init();
    if (maybe_error.has_value()) {
        printf("Could not initalize the http client: %s\n",
               maybe_error.value().second.c_str());
    }
    _buffered_data.reserve(capacity);
}

void SimpleSlam::BufferedHTTPClient::begin_processing() {
    while (true) {
        _mutex.lock();
        while (_buffered_data.size() < _capacity) {
            _cond_var.wait();
        }

        std::vector<std::vector<double>> spatials;
        std::vector<std::vector<double>> positions;
        for (auto& data_point : _buffered_data) {
            spatials.push_back({data_point.spatial_point.get_x(),
                                data_point.spatial_point.get_y()});
            positions.push_back({data_point.position_point.get_x(),
                                 data_point.position_point.get_y()});
        }
        _buffered_data.clear();
        _mutex.unlock();
        JSON data;

        data.add("board_id", "b1")
            .add_list("spatials", spatials)
            .add_list("positions", positions);

        printf("Hitting here\n");
        std::optional<HttpClient::error_t> maybe_error =
            _http_client.post_request(_host, "/collect", data);
        if (maybe_error.has_value()) {
            printf("Encountered Error in Buffered HTTP Client: %s\n",
                   maybe_error.value().second.c_str());
        }
    }
}

void SimpleSlam::BufferedHTTPClient::add_data(point_data_t const& data) {
    _mutex.lock();
    if (_buffered_data.size() < _capacity) {
        _buffered_data.push_back(data);
    }
    if (_buffered_data.size() >= _capacity) {
        _cond_var.notify_one();
    }
    _mutex.unlock();
}