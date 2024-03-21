#pragma once

#include <any>
#include <functional>
#include <type_traits>
#include <typeindex>
#include <typeinfo>
#include <unordered_map>

namespace SimpleSlam {

template <class R>
class DataBuilder {
   private:
    std::unordered_map<std::type_index, std::function<R(std::any const&)>>
        _visitors;

   public:
    DataBuilder(
        std::unordered_map<std::type_index,
                           std::function<R(std::any const&)>> const&& visitors)
        : _visitors(visitors){};

    template <class T, class F>
    inline void register_visitor(F const& f) {
        auto itr = _visitors.find(std::type_index(typeid(T)));
        if (itr == _visitors.end()) {
            _visitors.insert(to_visitor<T>(f));
        }
    }

    inline R build(const std::any& a) {
        const auto visitor_itr = _visitors.find(std::type_index(a.type()));
        if (visitor_itr != _visitors.cend()) {
            return visitor_itr->second(a);
        }

        return "";
    }

    template <class T, class F>
    static inline std::pair<const std::type_index,
                            std::function<R(std::any const&)>>
    to_visitor(F const& f) {
        return {std::type_index(typeid(T)), [g = f](std::any const& a) -> R {
                    if constexpr (std::is_void_v<T>)
                        return g();
                    else
                        return g(std::any_cast<T const&>(a));
                }};
    }
};

}  // namespace SimpleSlam
