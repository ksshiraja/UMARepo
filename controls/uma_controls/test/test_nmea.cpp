// Copyright 2019 UM::Autonomy

#include <string>
#include "ros/ros.h"
#include "uma_controls/NMEA_0183.h"
#include <gtest/gtest.h>

const double PI = 3.14159;

TEST(TestLib, test_emptystring)
{
    std::string str = "";
    try
    {
        NMEAMessage nmea(str);
        EXPECT_TRUE(false);
    }
    catch (const ControlsException &e)
    {
        EXPECT_TRUE(true);
    }
}

TEST(TestLib, test_multipleFlags)
{
    std::string str = "$PSRFS,quaternion,get,GLOM*34";
    NMEAMessage nmea(str);

    EXPECT_EQ(nmea.num_flags(), 4);
    EXPECT_EQ(nmea.num_parameters(), 0);

    EXPECT_EQ(nmea.get_command(), "$PSRFS");
    EXPECT_EQ(nmea.flag_present("quaternion"), true);
    EXPECT_EQ(nmea.flag_present("get"), true);
    EXPECT_EQ(nmea.flag_present("GLOM"), true);
}

TEST(TestLib, test_parseGyro)
{
    std::string str = "$PSPA,Gx=165.974,Gy=285.613,Gz=-168.670*31";
    NMEAMessage nmea(str);


    EXPECT_EQ(nmea.num_parameters(), 3);
    EXPECT_EQ(nmea.num_flags(), 1);

    EXPECT_EQ(nmea.get_command(), "$PSPA");
    EXPECT_EQ(nmea.get_parameter("Gx"), "165.974");
    EXPECT_EQ(nmea.get_parameter("Gy"), "285.613");
    EXPECT_EQ(nmea.get_parameter("Gz"), "-168.670");
}


TEST(TestLib, test_parseAcceleration)
{
    std::string str = "$PSPA,Ax=-70,Ay=76,Az=995,At=1000*02";
    NMEAMessage nmea(str);


    EXPECT_EQ(nmea.num_parameters(), 4);
    EXPECT_EQ(nmea.num_flags(), 1);

    EXPECT_EQ(nmea.get_command(), "$PSPA");
    EXPECT_EQ(nmea.get_parameter("Ax"), "-70");
    EXPECT_EQ(nmea.get_parameter("Ay"), "76");
    EXPECT_EQ(nmea.get_parameter("Az"), "995");
    EXPECT_EQ(nmea.get_parameter("At"), "1000");
}

TEST(TestLib, test_parseQuaternion)
{
    std::string str = "$PSPA,QUATw=0.314214,x=0.007481,y=-0.034541,z=-0.948694*0D";
    NMEAMessage nmea(str);


    EXPECT_EQ(nmea.num_parameters(), 4);
    EXPECT_EQ(nmea.num_flags(), 1);

    EXPECT_EQ(nmea.get_command(), "$PSPA");
    EXPECT_EQ(nmea.get_parameter("QUATw"), "0.314214");
    EXPECT_EQ(nmea.get_parameter("x"), "0.007481");
    EXPECT_EQ(nmea.get_parameter("y"), "-0.034541");
    EXPECT_EQ(nmea.get_parameter("z"), "-0.948694");
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    return RUN_ALL_TESTS();
}
