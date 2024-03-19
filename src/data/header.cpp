#include "data/header.h"

SimpleSlam::HeaderBuilder SimpleSlam::Header::_builder(
    {HeaderBuilder::to_visitor<std::unordered_map<std::string, std::string>>(
        [](std::unordered_map<std::string, std::string> map) -> std::string {
            std::stringstream ss;
            for (auto& itr : map) {
                if (itr.first != "Request Type") {
                    ss << itr.first << ": " << itr.second;
                    ss << "\r\n";
                } else {
                    std::string temp = ss.str();
                    ss.seekp(0);
                    ss << itr.second << "\r\n" << temp;
                }
            }
            return ss.str();
        })});

SimpleSlam::Header::Header() : _fields() {}

SimpleSlam::Header::Header(
    std::unordered_map<std::string, std::string> const&& fields)
    : _fields(fields) {}

SimpleSlam::Header& SimpleSlam::Header::add(std::string field,
                                            std::string value) {
    _fields.insert(std::make_pair(field, value));
    return *this;
}

SimpleSlam::Header& SimpleSlam::Header::request_type(
    HTTPRequestType request_type, std::string endpoint) {
    switch (request_type) {
        case HTTPRequestType::GET:
            _fields.insert({"Request Type", "GET " + endpoint + " HTTP/1.1"});
            break;
        case HTTPRequestType::POST:
            _fields.insert({"Request Type", "POST " + endpoint + " HTTP/1.1"});
            break;
        case HTTPRequestType::DELETE:
            _fields.insert({"Request Type", "DELETE " + endpoint + " HTTP/1.1"});
            break;
    }
    return *this;
}

std::string SimpleSlam::Header::build() { return _builder.build(_fields); }
