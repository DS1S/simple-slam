#include "data/json.h"

SimpleSlam::JSON::JSON() : _fields() {}

SimpleSlam::JSON::JSON(
    std::unordered_map<std::string, std::any> const&& fields)
    : _fields(fields) {}

SimpleSlam::JSON& SimpleSlam::JSON::add(std::string field,
                                                          std::any value) {
    _fields.insert(std::make_pair(field, value));
    return *this;
}

std::string SimpleSlam::JSON::build() { return _builder.build(_fields); }