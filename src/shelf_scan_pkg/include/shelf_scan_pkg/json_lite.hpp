// 极简 JSON 解析 / 生成（D 题机载端专用）
//
// 机载端只与地面站交换一行一条的 JSON（见 groundstation/PROTOCOL.md），
// 为了避免在香橙派上额外安装 nlohmann-json 之类的依赖，这里自带一个小型
// JSON 实现：支持对象、数组、字符串（含 \uXXXX 转义与 UTF-8 原文）、
// 数字、布尔、null。它是 DOM 式的，消息都很小，够用且行为可预期。
#pragma once

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

namespace shelf_scan
{
namespace json
{

class Value
{
public:
  enum class Type { Null, Bool, Number, String, Array, Object };
  using Array = std::vector<Value>;
  using Member = std::pair<std::string, Value>;
  using Object = std::vector<Member>;

  Value() = default;

  explicit Value(bool value) : type_(Type::Bool), bool_(value) {}
  explicit Value(double value) : type_(Type::Number), num_(value) {}
  explicit Value(int value) : type_(Type::Number), num_(static_cast<double>(value)) {}
  explicit Value(const std::string & value) : type_(Type::String), str_(value) {}
  explicit Value(const char * value) : type_(Type::String), str_(value == nullptr ? "" : value) {}

  static Value array() { Value v; v.type_ = Type::Array; return v; }
  static Value object() { Value v; v.type_ = Type::Object; return v; }

  Type type() const { return type_; }
  bool is_null() const { return type_ == Type::Null; }
  bool is_bool() const { return type_ == Type::Bool; }
  bool is_number() const { return type_ == Type::Number; }
  bool is_string() const { return type_ == Type::String; }
  bool is_array() const { return type_ == Type::Array; }
  bool is_object() const { return type_ == Type::Object; }

  bool as_bool(bool fallback = false) const
  {
    if (type_ == Type::Bool) return bool_;
    if (type_ == Type::Number) return num_ != 0.0;
    return fallback;
  }

  double as_double(double fallback = 0.0) const
  {
    if (type_ == Type::Number) return num_;
    if (type_ == Type::String) {
      try {
        return std::stod(str_);
      } catch (...) {
        return fallback;
      }
    }
    if (type_ == Type::Bool) return bool_ ? 1.0 : 0.0;
    return fallback;
  }

  long as_int(long fallback = 0) const
  {
    if (type_ == Type::Number) return static_cast<long>(std::llround(num_));
    if (type_ == Type::String) {
      try {
        return std::stol(str_);
      } catch (...) {
        return fallback;
      }
    }
    return fallback;
  }

  std::string as_string(const std::string & fallback = std::string()) const
  {
    if (type_ == Type::String) return str_;
    if (type_ == Type::Number) return formatNumber(num_);
    if (type_ == Type::Bool) return bool_ ? "true" : "false";
    return fallback;
  }

  const Array & items() const { return arr_; }
  Array & items() { return arr_; }

  const Object & members() const { return obj_; }
  Object & members() { return obj_; }

  std::size_t size() const
  {
    if (type_ == Type::Array) return arr_.size();
    if (type_ == Type::Object) return obj_.size();
    return 0;
  }

  bool has(const std::string & key) const { return find(key) != nullptr; }

  const Value * find(const std::string & key) const
  {
    if (type_ != Type::Object) return nullptr;
    for (const auto & member : obj_) {
      if (member.first == key) return &member.second;
    }
    return nullptr;
  }

  // 只读访问：键不存在时返回一个 static 的 null 值，便于链式取值。
  const Value & operator[](const std::string & key) const
  {
    const Value * found = find(key);
    if (found != nullptr) return *found;
    static const Value kNull;
    return kNull;
  }

  const Value & operator[](std::size_t index) const
  {
    if (type_ == Type::Array && index < arr_.size()) return arr_[index];
    static const Value kNull;
    return kNull;
  }

  Value & at(const std::string & key)
  {
    if (type_ != Type::Object) type_ = Type::Object;
    for (auto & member : obj_) {
      if (member.first == key) return member.second;
    }
    obj_.emplace_back(key, Value());
    return obj_.back().second;
  }

  void set(const std::string & key, const Value & value) { at(key) = value; }

  void push_back(const Value & value)
  {
    if (type_ != Type::Array) type_ = Type::Array;
    arr_.push_back(value);
  }

  std::string dump() const
  {
    std::string out;
    dumpTo(out);
    return out;
  }

  static std::string formatNumber(double value)
  {
    if (!std::isfinite(value)) return "0";
    if (value == std::floor(value) && std::fabs(value) < 1e15) {
      char buffer[32];
      std::snprintf(buffer, sizeof(buffer), "%.0f", value);
      return std::string(buffer);
    }
    char buffer[40];
    std::snprintf(buffer, sizeof(buffer), "%.6g", value);
    return std::string(buffer);
  }

private:
  void dumpTo(std::string & out) const
  {
    switch (type_) {
      case Type::Null:
        out += "null";
        break;
      case Type::Bool:
        out += bool_ ? "true" : "false";
        break;
      case Type::Number:
        out += formatNumber(num_);
        break;
      case Type::String:
        out += '"';
        appendEscaped(out, str_);
        out += '"';
        break;
      case Type::Array: {
        out += '[';
        for (std::size_t i = 0; i < arr_.size(); ++i) {
          if (i != 0) out += ',';
          arr_[i].dumpTo(out);
        }
        out += ']';
        break;
      }
      case Type::Object: {
        out += '{';
        for (std::size_t i = 0; i < obj_.size(); ++i) {
          if (i != 0) out += ',';
          out += '"';
          appendEscaped(out, obj_[i].first);
          out += "\":";
          obj_[i].second.dumpTo(out);
        }
        out += '}';
        break;
      }
    }
  }

  static void appendEscaped(std::string & out, const std::string & text)
  {
    for (unsigned char ch : text) {
      switch (ch) {
        case '"': out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        case '\b': out += "\\b"; break;
        case '\f': out += "\\f"; break;
        default:
          if (ch < 0x20) {
            char buffer[8];
            std::snprintf(buffer, sizeof(buffer), "\\u%04x", static_cast<unsigned>(ch));
            out += buffer;
          } else {
            out += static_cast<char>(ch);
          }
      }
    }
  }

  Type type_ = Type::Null;
  bool bool_ = false;
  double num_ = 0.0;
  std::string str_;
  Array arr_;
  Object obj_;
};

namespace detail
{

class Parser
{
public:
  explicit Parser(const std::string & text) : text_(text) {}

  bool parse(Value & out)
  {
    skipSpace();
    if (!parseValue(out, 0)) return false;
    skipSpace();
    pos_ = text_.size();  // 允许尾部有换行等空白
    return true;
  }

private:
  static constexpr int kMaxDepth = 32;

  bool eof() const { return pos_ >= text_.size(); }
  char peek() const { return eof() ? '\0' : text_[pos_]; }

  void skipSpace()
  {
    while (!eof()) {
      const char ch = text_[pos_];
      if (ch == ' ' || ch == '\t' || ch == '\n' || ch == '\r') {
        ++pos_;
      } else {
        break;
      }
    }
  }

  bool consume(const char * literal)
  {
    const std::size_t length = std::string(literal).size();
    if (text_.compare(pos_, length, literal) != 0) return false;
    pos_ += length;
    return true;
  }

  bool parseValue(Value & out, int depth)
  {
    if (depth > kMaxDepth) return false;
    skipSpace();
    if (eof()) return false;
    const char ch = peek();
    if (ch == '{') return parseObject(out, depth);
    if (ch == '[') return parseArray(out, depth);
    if (ch == '"') {
      std::string text;
      if (!parseString(text)) return false;
      out = Value(text);
      return true;
    }
    if (ch == 't') {
      if (!consume("true")) return false;
      out = Value(true);
      return true;
    }
    if (ch == 'f') {
      if (!consume("false")) return false;
      out = Value(false);
      return true;
    }
    if (ch == 'n') {
      if (!consume("null")) return false;
      out = Value();
      return true;
    }
    return parseNumber(out);
  }

  bool parseObject(Value & out, int depth)
  {
    ++pos_;  // '{'
    out = Value::object();
    skipSpace();
    if (peek() == '}') {
      ++pos_;
      return true;
    }
    while (true) {
      skipSpace();
      std::string key;
      if (!parseString(key)) return false;
      skipSpace();
      if (peek() != ':') return false;
      ++pos_;
      Value value;
      if (!parseValue(value, depth + 1)) return false;
      out.members().emplace_back(key, value);
      skipSpace();
      const char ch = peek();
      if (ch == ',') {
        ++pos_;
        continue;
      }
      if (ch == '}') {
        ++pos_;
        return true;
      }
      return false;
    }
  }

  bool parseArray(Value & out, int depth)
  {
    ++pos_;  // '['
    out = Value::array();
    skipSpace();
    if (peek() == ']') {
      ++pos_;
      return true;
    }
    while (true) {
      Value value;
      if (!parseValue(value, depth + 1)) return false;
      out.push_back(value);
      skipSpace();
      const char ch = peek();
      if (ch == ',') {
        ++pos_;
        continue;
      }
      if (ch == ']') {
        ++pos_;
        return true;
      }
      return false;
    }
  }

  bool parseString(std::string & out)
  {
    if (peek() != '"') return false;
    ++pos_;
    out.clear();
    while (!eof()) {
      const unsigned char ch = static_cast<unsigned char>(text_[pos_]);
      if (ch == '"') {
        ++pos_;
        return true;
      }
      if (ch == '\\') {
        ++pos_;
        if (eof()) return false;
        const char esc = text_[pos_++];
        switch (esc) {
          case '"': out += '"'; break;
          case '\\': out += '\\'; break;
          case '/': out += '/'; break;
          case 'b': out += '\b'; break;
          case 'f': out += '\f'; break;
          case 'n': out += '\n'; break;
          case 'r': out += '\r'; break;
          case 't': out += '\t'; break;
          case 'u': {
            unsigned code = 0;
            if (!parseHex4(code)) return false;
            if (code >= 0xD800 && code <= 0xDBFF && pos_ + 1 < text_.size() &&
              text_[pos_] == '\\' && text_[pos_ + 1] == 'u')
            {
              std::size_t saved = pos_;
              pos_ += 2;
              unsigned low = 0;
              if (parseHex4(low) && low >= 0xDC00 && low <= 0xDFFF) {
                code = 0x10000 + ((code - 0xD800) << 10) + (low - 0xDC00);
              } else {
                pos_ = saved;
              }
            }
            appendUtf8(out, code);
            break;
          }
          default:
            return false;
        }
        continue;
      }
      if (ch < 0x20) return false;
      out += static_cast<char>(ch);
      ++pos_;
    }
    return false;
  }

  bool parseHex4(unsigned & code)
  {
    if (pos_ + 4 > text_.size()) return false;
    code = 0;
    for (int i = 0; i < 4; ++i) {
      const char ch = text_[pos_ + static_cast<std::size_t>(i)];
      unsigned digit = 0;
      if (ch >= '0' && ch <= '9') {
        digit = static_cast<unsigned>(ch - '0');
      } else if (ch >= 'a' && ch <= 'f') {
        digit = static_cast<unsigned>(ch - 'a' + 10);
      } else if (ch >= 'A' && ch <= 'F') {
        digit = static_cast<unsigned>(ch - 'A' + 10);
      } else {
        return false;
      }
      code = (code << 4) | digit;
    }
    pos_ += 4;
    return true;
  }

  static void appendUtf8(std::string & out, unsigned code)
  {
    if (code < 0x80) {
      out += static_cast<char>(code);
    } else if (code < 0x800) {
      out += static_cast<char>(0xC0 | (code >> 6));
      out += static_cast<char>(0x80 | (code & 0x3F));
    } else if (code < 0x10000) {
      out += static_cast<char>(0xE0 | (code >> 12));
      out += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
      out += static_cast<char>(0x80 | (code & 0x3F));
    } else {
      out += static_cast<char>(0xF0 | (code >> 18));
      out += static_cast<char>(0x80 | ((code >> 12) & 0x3F));
      out += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
      out += static_cast<char>(0x80 | (code & 0x3F));
    }
  }

  bool parseNumber(Value & out)
  {
    const std::size_t start = pos_;
    if (peek() == '-' || peek() == '+') ++pos_;
    bool digits = false;
    while (!eof() && text_[pos_] >= '0' && text_[pos_] <= '9') {
      ++pos_;
      digits = true;
    }
    if (!eof() && text_[pos_] == '.') {
      ++pos_;
      while (!eof() && text_[pos_] >= '0' && text_[pos_] <= '9') {
        ++pos_;
        digits = true;
      }
    }
    if (digits && !eof() && (text_[pos_] == 'e' || text_[pos_] == 'E')) {
      ++pos_;
      if (!eof() && (text_[pos_] == '-' || text_[pos_] == '+')) ++pos_;
      while (!eof() && text_[pos_] >= '0' && text_[pos_] <= '9') ++pos_;
    }
    if (!digits) return false;
    try {
      out = Value(std::stod(text_.substr(start, pos_ - start)));
    } catch (...) {
      return false;
    }
    return true;
  }

  const std::string & text_;
  std::size_t pos_ = 0;
};

}  // namespace detail

inline bool parse(const std::string & text, Value & out)
{
  detail::Parser parser(text);
  return parser.parse(out);
}

inline Value parse_or_null(const std::string & text)
{
  Value value;
  if (!parse(text, value)) return Value();
  return value;
}

}  // namespace json
}  // namespace shelf_scan
