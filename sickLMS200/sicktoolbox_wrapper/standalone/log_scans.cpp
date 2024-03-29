///////////////////////////////////////////////////////////////////////////////
// a little program that dumps LMS scans to disk in ASCII.
//
// Copyright (C) 2010, Morgan Quigley
//
// I am distributing this code under the BSD license:
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <csignal>
extern "C"
{
    // not everyone has <cstdint>
#include <stdint.h>
}
#include <cstdio>
#include <sicktoolbox/SickLMS2xx.hh>
#include <ros/time.h>
using namespace SickToolbox;
using namespace std;

bool got_ctrlc = false;
void ctrlc_handler(int)
{
    got_ctrlc = true;
}

int main(int argc, char** argv)
{
    if (argc != 4)
    {
        printf("Usage: log_scans DEVICE BAUD_RATE FILENAME\n");
        return 1;
    }
    FILE* log = fopen(argv[3], "w");
    if (!log)
    {
        fprintf(stderr, "couldn't open logfile %s\n", argv[3]);
        return 1;
    }
    string lms_dev = argv[1];
    SickLMS2xx::sick_lms_2xx_baud_t desired_baud = SickLMS2xx::StringToSickBaud(argv[2]);
    if (desired_baud == SickLMS2xx::SICK_BAUD_UNKNOWN)
    {
        printf("bad baud rate. must be one of {9600, 19200, 38400, 500000}\n");
        return 1;
    }
    signal(SIGINT, ctrlc_handler);
    uint32_t values[SickLMS2xx::SICK_MAX_NUM_MEASUREMENTS] = { 0 };
    uint32_t num_values = 0;
    SickLMS2xx sick_lms(lms_dev);
    try
    {
        sick_lms.Initialize(desired_baud);
    }
    catch (...)
    {
        printf("initialize failed! are you using the correct device path?\n");
    }
    try
    {
        while (!got_ctrlc)
        {
            sick_lms.GetSickScan(values, num_values);
            // print 12 ranges to the console
            int inc = num_values / 11;
            printf("%5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d\n",
                values[0], values[inc],
                values[2 * inc], values[3 * inc],
                values[4 * inc], values[5 * inc],
                values[6 * inc], values[7 * inc],
                values[8 * inc], values[9 * inc],
                values[10 * inc], values[num_values - 1]);
            // dump all these guys to disk
            fprintf(log, "%.6f ", ros::Time::now().toSec());
            for (unsigned i = 0; i < num_values; i++)
                fprintf(log, "%d ", values[i]);
            fprintf(log, "\n");
        }
    }
    catch (...)
    {
        printf("woah! error!\n");
    }
    try
    {
        sick_lms.Uninitialize();
    }
    catch (...)
    {
        printf("error during uninitialize\n");
        return 1;
    }
    fclose(log);
    printf("success.\n");
    return 0;
}
