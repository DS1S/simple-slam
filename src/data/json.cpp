#include "data/json.h"

SimpleSlam::JSONBuilder SimpleSlam::JSON::_builder(
    {JSONBuilder::to_visitor<int>([](int x) -> std::string {
         return std::to_string(x);
     }),
     JSONBuilder::to_visitor<float>([](float x) -> std::string {
         return std::to_string(x);
     }),
     JSONBuilder::to_visitor<double>([](float x) -> std::string {
         return std::to_string(x);
     }),
     JSONBuilder::to_visitor<char const*>([](char const* s) -> std::string {
         std::stringstream ss;
         ss << std::quoted(s);
         return ss.str();
     }),
     JSONBuilder::to_visitor<std::unordered_map<std::string, std::any>>(
         [](std::unordered_map<std::string, std::any> const& map) {
             std::stringstream ss;
             ss << "{";

             for (auto itr = map.begin(); itr != map.end();) {
                 ss << std::quoted(itr->first) << ":"
                    << JSON::_builder.build(itr->second);

                 itr++;
                 if (itr != map.end()) {
                     ss << ",";
                 }
             }
             ss << "}";
             std::string json_str = ss.str();
             ss.str("");
             ss << std::quoted(json_str);

             return ss.str();
         })});

SimpleSlam::JSON::JSON() : _fields() {}

SimpleSlam::JSON::JSON(std::unordered_map<std::string, std::any> const&& fields)
    : _fields(fields) {}

SimpleSlam::JSON& SimpleSlam::JSON::add(std::string field,
                                        std::any const& value) {
    _fields.insert(std::make_pair(field, value));
    return *this;
}

std::string SimpleSlam::JSON::build() { return _builder.build(_fields); }