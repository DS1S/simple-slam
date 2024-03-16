#pragma once

#include <any>
#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_map>

#include "data/builder.h"

namespace SimpleSlam {

namespace {

typedef SimpleSlam::DataBuilder<std::string> JSONBuilder;

static JSONBuilder _builder(
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
     JSONBuilder::to_visitor<std::vector<std::any>>(
         [](std::vector<std::any> const& vec) -> std::string {
            std::stringstream ss;
            ss << "[";
            for (auto itr = vec.begin(); itr != vec.end();) {
                ss << _builder.build(*itr);
                itr++;
                if (itr != vec.end()) {
                    ss << ",";
                }
            }
            ss << "]";
            return ss.str();
         }),
     JSONBuilder::to_visitor<std::unordered_map<std::string, std::any>>(
         [](std::unordered_map<std::string, std::any> const& map) {
             std::stringstream ss;
             ss << "{";

             for (auto itr = map.begin(); itr != map.end();) {
                 ss << std::quoted(itr->first) << ":"
                    << _builder.build(itr->second);

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
}  // namespace

class JSON {
   private:
    std::unordered_map<std::string, std::any> _fields;

   public:
    JSON();
    JSON(std::unordered_map<std::string, std::any> const&& fields);
    JSON& add(std::string field, std::any value);
    std::string build();
};

}  // namespace SimpleSlam