#pragma once

namespace aos {
namespace testing {

// Returns the random seed to use for testing.
//
// This is ${TEST_RANDOM_SEED} if it is set or 1.
int RandomSeed();

}  // namespace testing
}  // namespace aos

