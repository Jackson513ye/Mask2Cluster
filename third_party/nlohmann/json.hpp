// SPDX-License-Identifier: MIT

#pragma once

#include <cctype>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

// This is a lightweight compatibility shim that offers a very small subset of
// the nlohmann::json API sufficient for the current mask2cluster prototype.
// It supports parsing JSON objects, arrays, strings, numbers, booleans, and
// null values. Only the methods required by the project are implemented. For a
// full-featured solution replace this file with the official json.hpp release
// from https://github.com/nlohmann/json.

namespace nlohmann {

class json {
 public:
	enum class value_t { null, boolean, number, string, object, array };
	using object_t = std::map<std::string, json>;
	using array_t = std::vector<json>;
	using string_t = std::string;

	json() : type_(value_t::null), number_(0.0), boolean_(false) {}
	json(double number) : type_(value_t::number), number_(number), boolean_(false) {}
	json(bool boolean) : type_(value_t::boolean), number_(0.0), boolean_(boolean) {}
	json(string_t string) : type_(value_t::string), number_(0.0), boolean_(false), string_(std::move(string)) {}
	json(object_t object) : type_(value_t::object), number_(0.0), boolean_(false), object_(std::move(object)) {}
	json(array_t array) : type_(value_t::array), number_(0.0), boolean_(false), array_(std::move(array)) {}

	bool is_object() const { return type_ == value_t::object; }

	bool contains(const std::string& key) const {
		if (!is_object()) {
			return false;
		}
		return object_.find(key) != object_.end();
	}

	json& operator[](const std::string& key) {
		ensure_object();
		return object_[key];
	}

	const json& operator[](const std::string& key) const {
		if (!is_object()) {
			throw std::out_of_range("json value is not an object");
		}
		return object_.at(key);
	}

	const json& at(const std::string& key) const { return (*this)[key]; }

	template <typename T>
	T get() const {
		static_assert(std::is_same<T, float>::value || std::is_same<T, double>::value ||
											std::is_same<T, int>::value,
									"minimal json shim only supports float/double/int conversions");
		if (type_ == value_t::number) {
			if constexpr (std::is_same<T, float>::value) {
				return static_cast<float>(number_);
			} else if constexpr (std::is_same<T, double>::value) {
				return number_;
			} else {
				return static_cast<int>(number_);
			}
		}
		if (type_ == value_t::boolean) {
			if constexpr (std::is_same<T, float>::value) {
				return boolean_ ? 1.0f : 0.0f;
			} else if constexpr (std::is_same<T, double>::value) {
				return boolean_ ? 1.0 : 0.0;
			} else {
				return boolean_ ? 1 : 0;
			}
		}
		throw std::runtime_error("json::get conversion failed: incompatible type");
	}

	static json parse(std::istream& is) {
		skip_ws(is);
		json result = parse_value(is);
		skip_ws(is);
		return result;
	}

	friend std::istream& operator>>(std::istream& is, json& value) {
		value = parse(is);
		return is;
	}

 private:
	static void skip_ws(std::istream& is) {
		while (true) {
			const int ch = is.peek();
			if (ch == EOF || !std::isspace(static_cast<unsigned char>(ch))) {
				break;
			}
			is.get();
		}
	}

	static json parse_value(std::istream& is) {
		const int ch = is.peek();
		if (ch == EOF) {
			throw std::runtime_error("Unexpected end of input while parsing JSON");
		}
		switch (ch) {
			case 'n':
				return parse_null(is);
			case 't':
			case 'f':
				return parse_boolean(is);
			case '"':
				return json(parse_string(is));
			case '[':
				return parse_array(is);
			case '{':
				return parse_object(is);
			default:
				if (ch == '-' || std::isdigit(static_cast<unsigned char>(ch))) {
					return parse_number(is);
				}
				throw std::runtime_error("Invalid character in JSON input");
		}
	}

	static json parse_null(std::istream& is) {
		const std::string token = read_literal(is, 4);
		if (token != "null") {
			throw std::runtime_error("Invalid token while parsing null");
		}
		return json();
	}

	static json parse_boolean(std::istream& is) {
		const int ch = is.peek();
		if (ch == 't') {
			const std::string token = read_literal(is, 4);
			if (token != "true") {
				throw std::runtime_error("Invalid token while parsing true");
			}
			return json(true);
		}
		const std::string token = read_literal(is, 5);
		if (token != "false") {
			throw std::runtime_error("Invalid token while parsing false");
		}
		return json(false);
	}

	static json parse_number(std::istream& is) {
		std::string buffer;
		const auto push = [&](int ch) {
			buffer.push_back(static_cast<char>(ch));
			is.get();
		};

		if (is.peek() == '-') {
			push('-');
		}

		if (!std::isdigit(static_cast<unsigned char>(is.peek()))) {
			throw std::runtime_error("Invalid number in JSON");
		}

		if (is.peek() == '0') {
			push('0');
			if (std::isdigit(static_cast<unsigned char>(is.peek()))) {
				throw std::runtime_error("Leading zeros not permitted in JSON numbers");
			}
		} else {
			while (std::isdigit(static_cast<unsigned char>(is.peek()))) {
				push(is.peek());
			}
		}

		if (is.peek() == '.') {
			push('.');
			if (!std::isdigit(static_cast<unsigned char>(is.peek()))) {
				throw std::runtime_error("At least one digit required after decimal point");
			}
			while (std::isdigit(static_cast<unsigned char>(is.peek()))) {
				push(is.peek());
			}
		}

		if (is.peek() == 'e' || is.peek() == 'E') {
			push(is.peek());
			if (is.peek() == '+' || is.peek() == '-') {
				push(is.peek());
			}
			if (!std::isdigit(static_cast<unsigned char>(is.peek()))) {
				throw std::runtime_error("Exponent must contain digits");
			}
			while (std::isdigit(static_cast<unsigned char>(is.peek()))) {
				push(is.peek());
			}
		}

		double value = 0.0;
		try {
			value = std::stod(buffer);
		} catch (...) {
			throw std::runtime_error("Failed to convert JSON number");
		}
		return json(value);
	}

	static string_t parse_string(std::istream& is) {
		if (is.get() != '"') {
			throw std::runtime_error("Expected opening quote for JSON string");
		}

		string_t result;
		while (true) {
			const int ch = is.get();
			if (ch == EOF) {
				throw std::runtime_error("Unterminated string in JSON input");
			}
			if (ch == '"') {
				break;
			}
			if (ch == '\\') {
				const int esc = is.get();
				if (esc == EOF) {
					throw std::runtime_error("Invalid escape sequence in JSON string");
				}
				switch (esc) {
					case '"': result.push_back('"'); break;
					case '\\': result.push_back('\\'); break;
					case '/': result.push_back('/'); break;
					case 'b': result.push_back('\b'); break;
					case 'f': result.push_back('\f'); break;
					case 'n': result.push_back('\n'); break;
					case 'r': result.push_back('\r'); break;
					case 't': result.push_back('\t'); break;
					case 'u': {
						// Basic \uXXXX handling (no surrogate support)
						unsigned int code = 0;
						for (int i = 0; i < 4; ++i) {
							const int hex = is.get();
							if (hex == EOF || !std::isxdigit(static_cast<unsigned char>(hex))) {
								throw std::runtime_error("Invalid Unicode escape in JSON string");
							}
							code <<= 4;
							if (hex >= '0' && hex <= '9') {
								code += static_cast<unsigned int>(hex - '0');
							} else if (hex >= 'a' && hex <= 'f') {
								code += static_cast<unsigned int>(hex - 'a' + 10);
							} else {
								code += static_cast<unsigned int>(hex - 'A' + 10);
							}
						}
						if (code <= 0x7F) {
							result.push_back(static_cast<char>(code));
						} else if (code <= 0x7FF) {
							result.push_back(static_cast<char>(0xC0 | ((code >> 6) & 0x1F)));
							result.push_back(static_cast<char>(0x80 | (code & 0x3F)));
						} else {
							result.push_back(static_cast<char>(0xE0 | ((code >> 12) & 0x0F)));
							result.push_back(static_cast<char>(0x80 | ((code >> 6) & 0x3F)));
							result.push_back(static_cast<char>(0x80 | (code & 0x3F)));
						}
						break;
					}
					default:
						throw std::runtime_error("Unsupported escape character in JSON string");
				}
			} else {
				result.push_back(static_cast<char>(ch));
			}
		}
		return result;
	}

	static json parse_array(std::istream& is) {
		if (is.get() != '[') {
			throw std::runtime_error("Expected '[' to start JSON array");
		}
		array_t array;
		skip_ws(is);
		if (is.peek() == ']') {
			is.get();
			return json(array);
		}
		while (true) {
			array.push_back(parse_value(is));
			skip_ws(is);
			const int ch = is.get();
			if (ch == ']') {
				break;
			}
			if (ch != ',') {
				throw std::runtime_error("Expected ',' or ']' in JSON array");
			}
			skip_ws(is);
		}
		return json(array);
	}

	static json parse_object(std::istream& is) {
		if (is.get() != '{') {
			throw std::runtime_error("Expected '{' to start JSON object");
		}
		object_t object;
		skip_ws(is);
		if (is.peek() == '}') {
			is.get();
			return json(object);
		}
		while (true) {
			skip_ws(is);
			const string_t key = parse_string(is);
			skip_ws(is);
			if (is.get() != ':') {
				throw std::runtime_error("Expected ':' after JSON object key");
			}
			skip_ws(is);
			object.emplace(key, parse_value(is));
			skip_ws(is);
			const int ch = is.get();
			if (ch == '}') {
				break;
			}
			if (ch != ',') {
				throw std::runtime_error("Expected ',' or '}' in JSON object");
			}
			skip_ws(is);
		}
		return json(object);
	}

	static std::string read_literal(std::istream& is, std::size_t length) {
		std::string token(length, '\0');
		for (std::size_t i = 0; i < length; ++i) {
			const int ch = is.get();
			if (ch == EOF) {
				throw std::runtime_error("Unexpected end of input while reading literal");
			}
			token[i] = static_cast<char>(ch);
		}
		return token;
	}

	void ensure_object() {
		if (type_ == value_t::null) {
			type_ = value_t::object;
			object_.clear();
			return;
		}
		if (type_ != value_t::object) {
			throw std::runtime_error("json value is not an object");
		}
	}

	value_t type_;
	double number_;
	bool boolean_;
	string_t string_;
	object_t object_;
	array_t array_;
};

}  // namespace nlohmann
