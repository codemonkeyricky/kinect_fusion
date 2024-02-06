
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
    if (0)
    {
        Octree tree(8);

        for (auto i = 2; i < 8; ++i)
            for (auto j = 2; j < 8; ++j)
                for (auto k = 2; k < 8; ++k)
                    tree.update(i, j, k, 0);

        for (auto i = 0; i < 2; ++i)
            for (auto j = 0; j < 2; ++j)
                for (auto k = 0; k < 2; ++k)
                    tree.update(i, j, k, 1);

        EXPECT_EQ(tree.query_min(0, 1, 0, 1, 0, 1), 1);

        EXPECT_EQ(tree.query_min(0, 3, 0, 3, 0, 3), 0);
    }

    {
        Octree tree(128);

        // tree.update(5, 5, 5, 0);
        // EXPECT_EQ(tree.query_min(5, 5, 5, 5, 5, 5), 0);

        for (auto i = 0; i < 100; ++i)
        {
            tree.update(i, i, i, 0);
            std::cout << "### " << i << std::endl;
            EXPECT_EQ(tree.query_min(i, i, i, i, i, i), 0);
        }

        // tree.update(90, 73, 76, 0);
        // EXPECT_EQ(tree.query_min(90, 90, 73, 73, 76, 76), 0);
    }
}