#include <gtest/gtest.h>
#include <navex/costmap/CostMap.h>

// Declare a test
TEST(TestSuite, testCase1)
{
	ASSERT_TRUE(1 > 0);
	ASSERT_TRUE(2 > 0);
	ASSERT_TRUE(3 > 0);
}

// Declare another test
TEST(TestSuite, testCase2)
{
	ASSERT_TRUE(0 == 0);
	ASSERT_TRUE(0 == 0);
	ASSERT_TRUE(0 == 0);
	ASSERT_TRUE(1 == 1);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
