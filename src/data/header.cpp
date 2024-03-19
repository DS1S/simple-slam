#include "data/header.h"

SimpleSlam::HeaderBuilder SimpleSlam::Header::_builder(
    {HeaderBuilder::to_visitor<SimpleSlam::Header>(
        [](SimpleSlam::Header header) -> std::string {
            std::stringstream ss;
            ss << header._request << "\r\n";
            for (auto& itr : header._fields) {
                ss << itr.first << ": " << itr.second;
                ss << "\r\n";
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
            _request = "GET " + endpoint + " HTTP/1.1";
            break;
        case HTTPRequestType::POST:
            _request = "POST " + endpoint + " HTTP/1.1";
            break;
        case HTTPRequestType::DELETE:
            _request = "DELETE " + endpoint + " HTTP/1.1";
            break;
    }
    return *this;
}

std::string SimpleSlam::Header::build() { return _builder.build(*this); }
