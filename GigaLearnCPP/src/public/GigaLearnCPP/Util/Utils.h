#pragma once

#include "../Framework.h"

#include <nlohmann/json.hpp>

namespace GGL {
	namespace Utils {
		template <typename T>
		nlohmann::json MakeJSONArray(const std::vector<T> list) {
			auto result = nlohmann::json::array();

			for (T v : list) {
				if (isnan(v))
					RG_LOG("MakeJSONArray(): Failed to serialize JSON with NAN value (list size: " << list.size() << ")");
				result.push_back(v);
			}

			return result;
		}

		template <typename T>
		std::vector<T> MakeVecFromJSON(const nlohmann::json& json) {
			return json.get<std::vector<T>>();
		}

		std::set<int64_t> FindNumberedDirs(std::filesystem::path basePath);

		template <typename T>
		std::string NumToStr(T val) {
			// https://stackoverflow.com/a/7277333
			class comma_numpunct : public std::numpunct<char>
			{
			protected:
				virtual char do_thousands_sep() const {
					return ',';
				}

				virtual std::string do_grouping() const {
					return "\03";
				}
			};
			static std::locale commaLocale(std::locale(), new comma_numpunct());

			std::stringstream stream;
			stream.imbue(commaLocale);

			T valMag = val;
			if constexpr (std::is_signed<T>())
				valMag = abs(val);

			if ((valMag < 1e-3 && valMag > 0) || valMag >= 1e11) {
				stream << std::scientific << val;
			} else {
				if (val == (int64_t)val) {
					stream << (int64_t)val;
				} else {
					stream << std::fixed << std::setprecision(4) << val;
				}
			}

			return stream.str();
		}
	}
}