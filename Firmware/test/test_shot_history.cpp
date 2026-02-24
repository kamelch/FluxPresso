#include <unity.h>
#include "utils/shot_history.h"

static ShotHistory make_shot(float yield_g, uint32_t time_ms, float temp_c, float stability) {
    ShotHistory s{};
    s.timestamp = 0;
    s.yield_g = yield_g;
    s.total_time_ms = time_ms;
    s.avg_temp_c = temp_c;
    s.temp_stability_score = stability;
    return s;
}

static void test_shot_history_circular_buffer_and_load() {
    ShotHistoryManager mgr(3);
    TEST_ASSERT_TRUE(mgr.begin());

    mgr.clear();

    TEST_ASSERT_TRUE(mgr.saveShot(make_shot(10, 25000, 93.0f, 0.8f)));
    TEST_ASSERT_TRUE(mgr.saveShot(make_shot(11, 26000, 93.5f, 0.7f)));
    TEST_ASSERT_TRUE(mgr.saveShot(make_shot(12, 27000, 94.0f, 0.9f)));
    TEST_ASSERT_TRUE(mgr.saveShot(make_shot(13, 28000, 92.0f, 0.6f))); // overwrites oldest

    TEST_ASSERT_EQUAL_UINT8(3, mgr.getCount());

    ShotHistory s{};
    // Oldest should now be 11
    TEST_ASSERT_TRUE(mgr.loadShot(0, s));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 11.0f, s.yield_g);

    // Newest should be 13
    TEST_ASSERT_TRUE(mgr.getLatestShot(s));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 13.0f, s.yield_g);
}

static void test_shot_history_stats() {
    ShotHistoryManager mgr(5);
    TEST_ASSERT_TRUE(mgr.begin());
    mgr.clear();

    mgr.saveShot(make_shot(20, 20000, 93.0f, 0.5f));
    mgr.saveShot(make_shot(30, 30000, 95.0f, 0.7f));
    mgr.saveShot(make_shot(40, 40000, 94.0f, 0.9f));

    auto st = mgr.calculateStats();
    TEST_ASSERT_EQUAL_UINT8(3, st.shot_count);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 30.0f, st.avg_yield_g);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 30.0f, st.avg_time_s);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 94.0f, st.avg_temp_c);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.7f, st.avg_stability_score);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f, st.min_yield_g);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 40.0f, st.max_yield_g);
}

void run_shot_history_tests() {
    RUN_TEST(test_shot_history_circular_buffer_and_load);
    RUN_TEST(test_shot_history_stats);
}
