#include <unity.h>
#include "thermistor.h"

void test_exact_table_value() {
    TEST_ASSERT_FLOAT_WITHIN(0.01, 110.0f, computeTemp(513));
}

void test_interpolated_value() {
    float expected = 110.0f + (100.0f - 110.0f) * (550.0f - 513.0f) / (588.0f - 513.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01, expected, computeTemp(550));
}

void test_out_of_range() {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, computeTemp(2000));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_exact_table_value);
    RUN_TEST(test_interpolated_value);
    RUN_TEST(test_out_of_range);
    return UNITY_END();
}
