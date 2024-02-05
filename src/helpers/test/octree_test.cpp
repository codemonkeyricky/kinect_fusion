
#include <gtest/gtest.h>

#include "octree.hpp"

// Demonstrate some basic assertions.
// TEST(HelloTest, BasicAssertions)
// {
//     Octree tree(8);
//     for (auto i = 0; i < 2; ++i)
//         for (auto j = 0; j < 2; ++j)
//             for (auto k = 0; k < 2; ++k)
//                 tree.update(i, j, k, 1);
//     EXPECT_EQ(tree.sum(1, 1, 0, 1, 0, 1), 4);
//     EXPECT_EQ(tree.sum(0, 1, 0, 1, 0, 1), 8);

//     for (auto i = 0; i < 2; ++i)
//         for (auto j = 0; j < 2; ++j)
//             for (auto k = 0; k < 2; ++k)
//                 tree.update(i, j, k, 0);

//     EXPECT_EQ(tree.sum(0, 1, 0, 1, 0, 1), 0);

//     tree.update(1, 1, 1, 1);
//     EXPECT_EQ(tree.sum(0, 3, 0, 3, 0, 3), 1);

//     EXPECT_EQ(tree.sum(1, 1, 1, 1, 1, 1), 1);

//     EXPECT_EQ(tree.sum(0, 0, 0, 0, 0, 0), 0);

//     EXPECT_EQ(tree.sum(0, 1, 0, 1, 0, 1), 1);
// }

TEST(floatTest, BasicAssertions)
{
    Octree tree(8);
    for (auto i = 0; i < 2; ++i)
        for (auto j = 0; j < 2; ++j)
            for (auto k = 0; k < 2; ++k)
                tree.update(i, j, k, 1);
    EXPECT_EQ(tree.mmin(0, 1, 0, 1, 0, 1), 1);
    // EXPECT_EQ(tree.mmin(0, 1, 0, 1, 0, 1), 8);

    // for (auto i = 0; i < 2; ++i)
    //     for (auto j = 0; j < 2; ++j)
    //         for (auto k = 0; k < 2; ++k)
    //             tree.update(i, j, k, 0);

    // EXPECT_EQ(tree.sum(0, 1, 0, 1, 0, 1), 0);

    // tree.update(1, 1, 1, 1);
    // EXPECT_EQ(tree.sum(0, 3, 0, 3, 0, 3), 1);

    // EXPECT_EQ(tree.sum(1, 1, 1, 1, 1, 1), 1);

    // EXPECT_EQ(tree.sum(0, 0, 0, 0, 0, 0), 0);

    // EXPECT_EQ(tree.sum(0, 1, 0, 1, 0, 1), 1);
}