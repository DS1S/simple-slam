#pragma once

#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_map>

#include "data/builder.h"

namespace SimpleSlam {

namespace {
typedef SimpleSlam::DataBuilder<std::string> HeaderBuilder;

static HeaderBuilder _builder(
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
}  // namespace

class Header {
   private:
    std::unordered_map<std::string, std::string> _fields;

   public:
    Header();
    Header(std::unordered_map<std::string, std::string> const&& fields);
    Header& add(std::string field, std::string value);
    std::string build();
};

}  // namespace
