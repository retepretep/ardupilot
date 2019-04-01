#pragma once

/*

Debug switches added by Peter

*/

#define ISDODEBUGPRINTOUTS              1       // normal printouts for debug purposes
#define ISDOREPEATEDGCSMESSAGE          1       // send an arbitrary message to gcs in a specific interval, to verify custom arducopter code works
#define ISDOVERBOSECSMAGPRINTOUTS       0       // print info for every CSMAG0 message sent
#define ISDOCRAZYDEBUGPRINTOUTS         0       // eg printouts in loops that are called often and will cause performance loss
#define ISDOABORTS                      0       // execute abort() to verify a particular code is executed
#define ISDOVERBOSEINITPRINTOUTS        00
#define ISDOBUFFERDEBUGPRINTOUTS        0       // for csmag state buffers
#define ISMONITORCHECKCSMAG             0       // print info about buffers and whether CSMAG is sent or not
#define ISUSESLOMOCSMAG                 00       // execute read_csmag and check_send_csmag slower than usual (-> SCHEDULER) to make debug
                                                // printouts more readable
#define ISDOVERBOSEUARTCHECK            0       // send repeated message via UARTC (TELEM1 on Pixhawk1)
#define ISDOMAGDATAINITREADUARTCHECK    00       // write out some status, if reading from UART worked at init
#define ISDOMAGDATAREADUARTCHECK        0       // write out some status, if reading from UART worked
#define ISPRINTTIMESTAMPREADCSMAG       0      // print a timestamp every time Copter::read_csmag() is called
#define ISDOINTERVALMAGDATAREADUARTCHECK    1   // same as ISDOMAGDATAREADUARTCHECK, but together with ISDOREPEATEDGCSMESSAGE

#define ISDOVERBOSEDEBUGPRINTOUTS       0
#define ISDOTEMPVERBOSEDEBUG            0       // do debug printouts for the most recent problem (then change it to ISDOVERBOSEDEBUGPRINTOUTS)

#define ISDOUARTDEBUG                   0
#define ISPRINTOUTNOUARTCONNECTIONVERBOSE       0   // print out full message
#define ISPRINTOUTNOUARTCONNECTIONSIMPLE        1   // just print out "X"



#define SLOMOFACTOR                     10      // check integer division in ArduCopter.cpp SCHEDULER, SLOMOFACTOR 50 is also ok
#define ISPRINTONGCS                    1       // print on GCS, sitl console or Mission Planner
#define ISPRINTMESSAGESINTOFILE         0       // print messages attempted so send into ardupilot-cs/MyLogs/msgLogs.txt
#define ISPRINTMESSAGESFULL             0       // print <timestamp>: <msgid>
#define ISPRINTMESSAGESSUMMARY          1       // print list of sent messages every interval
#define PRINTMESSAGESSUMMARYINTERVAL    10000   // every how many attempts shall the summary be printed?
#define GCSMESSAGEINTERVAL              ((int) 30E6)      // microseconds
//#define GCSMESSAGEINTERVAL              ((int) 5E6)      // microseconds

#define NUMBEROFCSMAGPRINTOUTS          5       // only print this much status data for each csmag0 message


// no need to change defines from here

#if ISPRINTONGCS
//#include <GCS_MAVLink/GCS.h> // added by Peter, for printing out messages to GCS, probably not necessary
#endif

static_assert(!(ISPRINTOUTNOUARTCONNECTIONSIMPLE && ISPRINTOUTNOUARTCONNECTIONVERBOSE), 
                "cannot have both warning messages for missing uart connection");