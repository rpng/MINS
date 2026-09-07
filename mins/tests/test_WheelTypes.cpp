// Covers the wheel type enum helpers: name round-trip, 2D/3D flavor, and measurement modality.
#include <gtest/gtest.h>
#include <string>

#include "update/wheel/WheelTypes.h"

using namespace mins;

TEST(WheelTypes, ToStringNamesEveryType) {
    EXPECT_STREQ(ToString(WheelType::Wheel2DAng), "Wheel2DAng");
    EXPECT_STREQ(ToString(WheelType::Wheel2DLin), "Wheel2DLin");
    EXPECT_STREQ(ToString(WheelType::Wheel2DCen), "Wheel2DCen");
    EXPECT_STREQ(ToString(WheelType::Wheel3DAng), "Wheel3DAng");
    EXPECT_STREQ(ToString(WheelType::Wheel3DLin), "Wheel3DLin");
    EXPECT_STREQ(ToString(WheelType::Wheel3DCen), "Wheel3DCen");
}

TEST(WheelTypes, ParseRoundTripsEveryType) {
    for (WheelType expected : ALL_WHEEL_TYPES) {
        WheelType parsed = WheelType::Wheel3DCen;
        ASSERT_TRUE(ParseWheelType(ToString(expected), parsed)) << ToString(expected);
        EXPECT_EQ(parsed, expected);
    }
}

TEST(WheelTypes, ParseRejectsUnknownNameAndLeavesOutputAlone) {
    WheelType parsed = WheelType::Wheel2DLin;
    EXPECT_FALSE(ParseWheelType("Wheel4DAng", parsed));
    EXPECT_FALSE(ParseWheelType("", parsed));
    EXPECT_FALSE(ParseWheelType("wheel2dang", parsed)); // names are case sensitive
    EXPECT_EQ(parsed, WheelType::Wheel2DLin);
}

TEST(WheelTypes, IsWheel3DSplitsOnTheFlavor) {
    EXPECT_FALSE(IsWheel3D(WheelType::Wheel2DAng));
    EXPECT_FALSE(IsWheel3D(WheelType::Wheel2DLin));
    EXPECT_FALSE(IsWheel3D(WheelType::Wheel2DCen));
    EXPECT_TRUE(IsWheel3D(WheelType::Wheel3DAng));
    EXPECT_TRUE(IsWheel3D(WheelType::Wheel3DLin));
    EXPECT_TRUE(IsWheel3D(WheelType::Wheel3DCen));
}

TEST(WheelTypes, ModalityIgnoresTheFlavor) {
    EXPECT_EQ(ModalityOf(WheelType::Wheel2DAng), WheelModality::Angular);
    EXPECT_EQ(ModalityOf(WheelType::Wheel3DAng), WheelModality::Angular);
    EXPECT_EQ(ModalityOf(WheelType::Wheel2DLin), WheelModality::Linear);
    EXPECT_EQ(ModalityOf(WheelType::Wheel3DLin), WheelModality::Linear);
    EXPECT_EQ(ModalityOf(WheelType::Wheel2DCen), WheelModality::Centered);
    EXPECT_EQ(ModalityOf(WheelType::Wheel3DCen), WheelModality::Centered);
}

TEST(WheelTypes, AllWheelTypesCoversTheEnum) {
    // Guards against a new enumerator being added without extending the table that
    // ParseWheelType walks, which would silently make the new name unparseable.
    EXPECT_EQ(sizeof(ALL_WHEEL_TYPES) / sizeof(ALL_WHEEL_TYPES[0]), 6u);
    for (WheelType type : ALL_WHEEL_TYPES) {
        EXPECT_STRNE(ToString(type), "");
    }
}

TEST(WheelData, SortsByTime) {
    WheelData early;
    early.time = 1.0;
    WheelData late;
    late.time = 2.0;
    EXPECT_TRUE(early < late);
    EXPECT_FALSE(late < early);
    EXPECT_FALSE(early < early);
}
