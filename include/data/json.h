#pragma once

#include <any>
#include <iomanip>
#include <sstream>
#include <type_traits>
#include <string>
#include <unordered_map>

#include "data/builder.h"

namespace SimpleSlam {

typedef SimpleSlam::DataBuilder<std::string> JSONBuilder;

class JSON {
   private:
    std::unordered_map<std::string, std::any> _fields;
    static JSONBuilder _builder;

   public:
    JSON();
    JSON(std::unordered_map<std::string, std::any> const&& fields);
    JSON& add(std::string field, std::any const& value);
    std::string build();
};

}  // namespace SimpleSlam
