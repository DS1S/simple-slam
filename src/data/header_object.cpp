#include "data/header.h"

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