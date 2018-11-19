// Created by Edoardo Ghini on 19/10/2018.
#include <stdio.h>
#include "gtest/gtest.h"

GTEST_API_ int main(int argc, char **argv) {
    printf("Running main() from %s\n", __FILE__);
      testing::InitGoogleTest(&argc, argv);
        return RUN_ALL_TESTS();
}

