#pragma once

#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_map>

#include "data/builder.h"

namespace SimpleSlam {

typedef SimpleSlam::DataBuilder<std::string> HeaderBuilder;

enum class HTTPRequestType {
    GET,
    POST,
    DELETE,
};

class Header {
   private:
    std::unordered_map<std::string, std::string> _fields;
    static HeaderBuilder _builder;

   public:
    Header();
    Header(std::unordered_map<std::string, std::string> const&& fields);
    Header& add(std::string field, std::string value);
    Header& request_type(HTTPRequestType request_type);
    std::string build();
};

}  // namespace SimpleSlam
