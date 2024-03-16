#pragma once

#include <any>
#include <iomanip>
#include <sstream>
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

    template <class T>
    JSON& add_list(std::string field, std::vector<T> list) {
        _builder.register_visitor<std::vector<T>>(
            [](std::vector<T> const& vec) -> std::string {
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
            });
        _fields.insert(std::make_pair(field, list));
        return *this;
    }
    std::string build();
};

}  // namespace SimpleSlam
