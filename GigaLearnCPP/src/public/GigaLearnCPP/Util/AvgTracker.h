#pragma once
#include "../Framework.h"

namespace GGL {
	struct AvgTracker {
		float total;
		uint64_t count;

		AvgTracker() {
			Reset();
		}

		// Returns 0 if no count
		float Get() const {
			if (count > 0) {
				return total / count;
			} else {
				return 0;
			}
		}

		void Add(float val) {
			if (!isnan(val)) {
				total += val;
				count++;
			}
		}

		AvgTracker& operator+=(float val) {
			Add(val);
			return *this;
		}

		void Add(float totalVal, uint64_t count) {
			if (!isnan(totalVal)) {
				total += totalVal;
				this->count += count;
			}
		}

		AvgTracker& operator+=(const AvgTracker& other) {
			Add(other.total, other.count);
			return *this;
		}

		void Reset() {
			total = 0;
			count = 0;
		}
	};

	// Thread-safe variant
	struct MutAvgTracker : AvgTracker {
		std::mutex mut = {};

		MutAvgTracker() {
			Reset();
		}

		// Returns 0 if no count
		float Get() {
			mut.lock();
			float result = AvgTracker::Get();
			mut.unlock();
			return result;
		}

		void Add(float val) {
			mut.lock();
			AvgTracker::Add(val);
			mut.unlock();
		}

		MutAvgTracker& operator+=(float val) {
			Add(val);
			return *this;
		}

		void Add(float totalVal, uint64_t count) {
			mut.lock();
			AvgTracker::Add(totalVal, count);
			mut.unlock();
		}

		MutAvgTracker& operator+=(const MutAvgTracker& other) {
			Add(other.total, other.count);
			return *this;
		}

		void Reset() {
			mut.lock();
			AvgTracker::Reset();
			mut.unlock();
		}
	};
}