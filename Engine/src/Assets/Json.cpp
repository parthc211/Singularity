#include "Assets/Json.h"

#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <format>

namespace SGE::Json {

namespace {

class Parser {
public:
    Parser(std::string_view text, std::string* error)
        : m_text(text), m_error(error) {}

    bool ParseDocument(Value& out) {
        SkipWs();
        if (!ParseValue(out)) return false;
        SkipWs();
        if (m_pos != m_text.size()) return Fail("trailing characters after JSON value");
        return true;
    }

private:
    std::string_view m_text;
    std::string*     m_error;
    size_t           m_pos   = 0;
    int              m_depth = 0;
    static constexpr int kMaxDepth = 128; // glTF nests ~5 deep; this guards garbage input

    bool Fail(const char* msg) {
        if (m_error && m_error->empty())
            *m_error = std::format("JSON error at byte {}: {}", m_pos, msg);
        return false;
    }

    char Peek() const { return m_pos < m_text.size() ? m_text[m_pos] : '\0'; }
    bool Eat(char c)  { if (Peek() != c) return false; ++m_pos; return true; }

    void SkipWs() {
        while (m_pos < m_text.size()) {
            const char c = m_text[m_pos];
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r') ++m_pos;
            else break;
        }
    }

    bool ParseValue(Value& out) {
        if (++m_depth > kMaxDepth) return Fail("nesting too deep");
        bool ok;
        switch (Peek()) {
            case '{': ok = ParseObject(out); break;
            case '[': ok = ParseArray(out);  break;
            case '"': out.Kind = Value::Type::String; ok = ParseString(out.Str); break;
            case 't': case 'f': ok = ParseKeyword(out); break;
            case 'n': ok = ParseKeyword(out); break;
            default:  ok = ParseNumber(out); break;
        }
        --m_depth;
        return ok;
    }

    bool ParseKeyword(Value& out) {
        auto match = [&](std::string_view kw) {
            if (m_text.substr(m_pos, kw.size()) != kw) return false;
            m_pos += kw.size();
            return true;
        };
        if (match("true"))  { out.Kind = Value::Type::Bool; out.Boolean = true;  return true; }
        if (match("false")) { out.Kind = Value::Type::Bool; out.Boolean = false; return true; }
        if (match("null"))  { out.Kind = Value::Type::Null; return true; }
        return Fail("invalid keyword");
    }

    bool ParseNumber(Value& out) {
        // Validate the JSON number grammar, then let strtod do the conversion.
        const size_t start = m_pos;
        Eat('-');
        if (!std::isdigit(static_cast<unsigned char>(Peek()))) return Fail("invalid number");
        while (std::isdigit(static_cast<unsigned char>(Peek()))) ++m_pos;
        if (Eat('.')) {
            if (!std::isdigit(static_cast<unsigned char>(Peek()))) return Fail("digit expected after '.'");
            while (std::isdigit(static_cast<unsigned char>(Peek()))) ++m_pos;
        }
        if (Peek() == 'e' || Peek() == 'E') {
            ++m_pos;
            if (Peek() == '+' || Peek() == '-') ++m_pos;
            if (!std::isdigit(static_cast<unsigned char>(Peek()))) return Fail("digit expected in exponent");
            while (std::isdigit(static_cast<unsigned char>(Peek()))) ++m_pos;
        }
        // strtod needs a terminated buffer; numbers are short, so copy.
        const std::string num(m_text.substr(start, m_pos - start));
        out.Kind   = Value::Type::Number;
        out.Number = std::strtod(num.c_str(), nullptr);
        return true;
    }

    static void AppendUtf8(std::string& s, uint32_t cp) {
        if (cp < 0x80) {
            s += char(cp);
        } else if (cp < 0x800) {
            s += char(0xC0 | (cp >> 6));
            s += char(0x80 | (cp & 0x3F));
        } else if (cp < 0x10000) {
            s += char(0xE0 | (cp >> 12));
            s += char(0x80 | ((cp >> 6) & 0x3F));
            s += char(0x80 | (cp & 0x3F));
        } else {
            s += char(0xF0 | (cp >> 18));
            s += char(0x80 | ((cp >> 12) & 0x3F));
            s += char(0x80 | ((cp >> 6) & 0x3F));
            s += char(0x80 | (cp & 0x3F));
        }
    }

    bool ParseHex4(uint32_t& out) {
        out = 0;
        for (int i = 0; i < 4; ++i) {
            const char c = Peek();
            uint32_t d;
            if (c >= '0' && c <= '9')      d = uint32_t(c - '0');
            else if (c >= 'a' && c <= 'f') d = uint32_t(c - 'a' + 10);
            else if (c >= 'A' && c <= 'F') d = uint32_t(c - 'A' + 10);
            else return Fail("invalid \\u escape");
            out = (out << 4) | d;
            ++m_pos;
        }
        return true;
    }

    bool ParseString(std::string& out) {
        if (!Eat('"')) return Fail("'\"' expected");
        out.clear();
        while (true) {
            if (m_pos >= m_text.size()) return Fail("unterminated string");
            const char c = m_text[m_pos++];
            if (c == '"') return true;
            if (c != '\\') { out += c; continue; }  // UTF-8 bytes pass through

            if (m_pos >= m_text.size()) return Fail("unterminated escape");
            const char e = m_text[m_pos++];
            switch (e) {
                case '"':  out += '"';  break;
                case '\\': out += '\\'; break;
                case '/':  out += '/';  break;
                case 'b':  out += '\b'; break;
                case 'f':  out += '\f'; break;
                case 'n':  out += '\n'; break;
                case 'r':  out += '\r'; break;
                case 't':  out += '\t'; break;
                case 'u': {
                    uint32_t cp;
                    if (!ParseHex4(cp)) return false;
                    // Surrogate pair -> single code point.
                    if (cp >= 0xD800 && cp <= 0xDBFF) {
                        if (!(Eat('\\') && Eat('u'))) return Fail("unpaired surrogate");
                        uint32_t lo;
                        if (!ParseHex4(lo)) return false;
                        if (lo < 0xDC00 || lo > 0xDFFF) return Fail("invalid low surrogate");
                        cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                    }
                    AppendUtf8(out, cp);
                    break;
                }
                default: return Fail("invalid escape character");
            }
        }
    }

    bool ParseArray(Value& out) {
        Eat('[');
        out.Kind = Value::Type::Array;
        SkipWs();
        if (Eat(']')) return true;
        while (true) {
            Value& item = out.Items.emplace_back();
            SkipWs();
            if (!ParseValue(item)) return false;
            SkipWs();
            if (Eat(']')) return true;
            if (!Eat(',')) return Fail("',' or ']' expected in array");
        }
    }

    bool ParseObject(Value& out) {
        Eat('{');
        out.Kind = Value::Type::Object;
        SkipWs();
        if (Eat('}')) return true;
        while (true) {
            SkipWs();
            std::string key;
            if (!ParseString(key)) return false;
            SkipWs();
            if (!Eat(':')) return Fail("':' expected after object key");
            SkipWs();
            Value& v = out.Members.emplace_back(std::move(key), Value{}).second;
            if (!ParseValue(v)) return false;
            SkipWs();
            if (Eat('}')) return true;
            if (!Eat(',')) return Fail("',' or '}' expected in object");
        }
    }
};

} // namespace

bool Parse(std::string_view text, Value& out, std::string* error)
{
    if (error) error->clear();
    out = Value{};
    return Parser(text, error).ParseDocument(out);
}

} // namespace SGE::Json
