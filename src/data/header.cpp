#include "data/header.h"

SimpleSlam::HeaderBuilder SimpleSlam::Header::_builder(
    {HeaderBuilder::to_visitor<std::unordered_map<std::string, std::string>>(
        [](std::unordered_map<std::string, std::string> map) -> std::string {
            std::stringstream ss;
            ss << "{";
            for (auto& itr : map) {
                ss << std::quoted(itr.first) << " : " << std::quoted(itr.second)
                   << ", ";
            }
            ss << "}";
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

std::string SimpleSlam::Header::build() { return _builder.build(_fields); }