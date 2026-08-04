#pragma once
// ---------------------------------------------------------------------------
// Minimal hand-written JSON parser (DOM-style) — exists to keep the glTF
// loader dependency-free. Supports the full JSON grammar: null/bool/number/
// string (with \uXXXX escapes incl. surrogate pairs), arrays, objects.
//
// Values are a tagged struct rather than a variant: a loader reads each value
// once, so compactness matters less than obvious code. Object members keep
// file order and are searched linearly (glTF objects are small).
// ---------------------------------------------------------------------------
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace SGE::Json {

class Value {
public:
    enum class Type { Null, Bool, Number, String, Array, Object };

    Type        Kind    = Type::Null;
    bool        Boolean = false;
    double      Number  = 0.0;
    std::string Str;
    std::vector<Value>                         Items;    // Array
    std::vector<std::pair<std::string, Value>> Members;  // Object

    bool IsNull()   const { return Kind == Type::Null;   }
    bool IsBool()   const { return Kind == Type::Bool;   }
    bool IsNumber() const { return Kind == Type::Number; }
    bool IsString() const { return Kind == Type::String; }
    bool IsArray()  const { return Kind == Type::Array;  }
    bool IsObject() const { return Kind == Type::Object; }

    // Object member lookup; nullptr if not an object or key absent.
    const Value* Find(std::string_view key) const {
        if (Kind != Type::Object) return nullptr;
        for (const auto& [k, v] : Members)
            if (k == key) return &v;
        return nullptr;
    }

    // Typed conveniences: missing or mistyped members yield the default.
    double GetNumber(std::string_view key, double def = 0.0) const {
        const Value* v = Find(key);
        return v && v->IsNumber() ? v->Number : def;
    }
    int GetInt(std::string_view key, int def = 0) const {
        return static_cast<int>(GetNumber(key, def));
    }
    bool GetBool(std::string_view key, bool def = false) const {
        const Value* v = Find(key);
        return v && v->IsBool() ? v->Boolean : def;
    }
    std::string_view GetString(std::string_view key, std::string_view def = {}) const {
        const Value* v = Find(key);
        return v && v->IsString() ? std::string_view(v->Str) : def;
    }

    // Array access.
    size_t Size() const { return Items.size(); }
    const Value& operator[](size_t i) const { return Items[i]; }
};

// Parses UTF-8 JSON text. Returns false and fills *error (message + byte
// offset) on malformed input.
bool Parse(std::string_view text, Value& out, std::string* error = nullptr);

} // namespace SGE::Json
