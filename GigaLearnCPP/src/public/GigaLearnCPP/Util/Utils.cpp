#include "Utils.h"

std::set<int64_t> GGL::Utils::FindNumberedDirs(std::filesystem::path basePath) {
	std::set<int64_t> results = {};

	if (!std::filesystem::exists(basePath))
		return results;

	for (auto entry : std::filesystem::directory_iterator(basePath)) {
		if (entry.is_directory()) {
			auto name = entry.path().filename();
			bool isNameAllNumbers = true;
			for (char c : name.string()) {
				if (!isdigit(c)) {
					isNameAllNumbers = false;
					break;
				}
			}
			if (!isNameAllNumbers)
				continue;

			results.insert(std::stoll(name.string()));
		}
	}

	return results;
}