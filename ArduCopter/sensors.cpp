#include "Copter.h"

// return barometric altitude in centimeters
void Copter::read_barometer(void)
{
    barometer.update();

    baro_alt = barometer.get_altitude() * 100.0f;
    baro_climbrate = barometer.get_climb_rate() * 100.0f;

    motors->set_air_density_ratio(barometer.get_air_density_ratio());
}

void Copter::init_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED
   rangefinder.init();
   rangefinder_state.alt_cm_filt.set_cutoff_frequency(RANGEFINDER_WPNAV_FILT_HZ);
   rangefinder_state.enabled = rangefinder.has_orientation(ROTATION_PITCH_270);
#endif
}

// return rangefinder altitude in centimeters
void Copter::read_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED
    rangefinder.update();

    if (rangefinder.num_sensors() > 0 &&
        should_log(MASK_LOG_CTUN)) {
        DataFlash.Log_Write_RFND(rangefinder);
    }

    rangefinder_state.alt_healthy = ((rangefinder.status_orient(ROTATION_PITCH_270) == RangeFinder::RangeFinder_Good) && (rangefinder.range_valid_count_orient(ROTATION_PITCH_270) >= RANGEFINDER_HEALTH_MAX));

    int16_t temp_alt = rangefinder.distance_cm_orient(ROTATION_PITCH_270);

 #if RANGEFINDER_TILT_CORRECTION == ENABLED
    // correct alt for angle of the rangefinder
    temp_alt = (float)temp_alt * MAX(0.707f, ahrs.get_rotation_body_to_ned().c.z);
 #endif

    rangefinder_state.alt_cm = temp_alt;

    // filter rangefinder for use by AC_WPNav
    uint32_t now = AP_HAL::millis();

    if (rangefinder_state.alt_healthy) {
        if (now - rangefinder_state.last_healthy_ms > RANGEFINDER_TIMEOUT_MS) {
            // reset filter if we haven't used it within the last second
            rangefinder_state.alt_cm_filt.reset(rangefinder_state.alt_cm);
        } else {
            rangefinder_state.alt_cm_filt.apply(rangefinder_state.alt_cm, 0.05f);
        }
        rangefinder_state.last_healthy_ms = now;
    }

    // send rangefinder altitude and health to waypoint navigation library
    wp_nav->set_rangefinder_alt(rangefinder_state.enabled, rangefinder_state.alt_healthy, rangefinder_state.alt_cm_filt.get());

#else
    rangefinder_state.enabled = false;
    rangefinder_state.alt_healthy = false;
    rangefinder_state.alt_cm = 0;
#endif
}

// return true if rangefinder_alt can be used
bool Copter::rangefinder_alt_ok()
{
    return (rangefinder_state.enabled && rangefinder_state.alt_healthy);
}

// added by peter {

// initialize csmag
// called in Copter::init_ardupilot(...) in system.cpp 
bool Copter::init_csmag(void) {

    // hal.console->printf("1. HELLO - hal.console->printf ok - 190329T1338+0100\n");
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "1. HELLO - hal.console->printf ok - 190329T1338+0100");

    is_first_csmag_message = true;

    if (IS_PRINT_WARNING_TIMESTAMPS) {
        hal.console->printf("init_csmag at AP_HAL::micros == %" PRIu64 "\n", AP_HAL::micros64());
    }

#if ISDOREPEATEDGCSMESSAGE
    // time_since_start = 0;                   // init counter
    // time_last_repeated_gcs_message = 0;
    // write message at the beginning
    time_since_start = AP_HAL::micros64();                   // init counter
    time_last_repeated_gcs_message = time_since_start - GCSMESSAGEINTERVAL;
#endif

#if ISDOVERBOSECSMAGPRINTOUTS    
    csmagDebugPrintCounter = 0;             // TODO: add conditioned declaration and use
#endif

    if (ISDOVERBOSEINITPRINTOUTS) {
        RangeFinder *rf = RangeFinder::get_singleton();
        printf("rf: %p\n", rf);
    }

    if (CSMAG_IS_USE_BUFFER_MODE) {
        // csmag state buffer init
        CsmagStateBuffer *csmag_state_buffer = CsmagStateBuffer::get_singleton();   // works

        if (ISDOBUFFERDEBUGPRINTOUTS || ISDOVERBOSEINITPRINTOUTS) {
            printf("(C10) csmag_state_buffer after get_singleton():\n");
            csmag_state_buffer->print_info();
        }
    }


    

    Csmag *csmag = Csmag::get_singleton();

    if (ISDOVERBOSEINITPRINTOUTS) {
        printf("(B10) Copter::init_csmag(): \n");
        printf("(B20) csmag: %p\n", csmag);
        printf("(B30) csmag->csmag_state: %p\n", csmag->csmag_state);
        printf("(B40) csmag->csmag_state->induction: %p\n", csmag->csmag_state->induction);
        printf("(B50) csmag->csmag_state->induction[0]: %d\n", csmag->csmag_state->induction[0]);
    }

    //printf("line %d ok.\n", __LINE__);                    // works
    //csmag->csmag_state->induction[0] = 148;               // works
    
    if (ISDOVERBOSEINITPRINTOUTS) {
        printf("line %d ok.\n", __LINE__);                    // works
        printf("(B10) Copter::init_csmag(): \n");
        printf("(B20) csmag->csmag_state->induction[0]: %d\n", csmag->csmag_state->induction[0]);
    }

    // some compilation error when using template class singleton:
    /*
    [527/527] Linking build/sitl/bin/arducopter
    ArduCopter/Copter.cpp.25.o: In function `Copter::Copter()':
    Copter.cpp:(.text._ZN6CopterC2Ev+0x11f): undefined reference to `RingBuffer<int>::RingBuffer(int)'
    ArduCopter/sensors.cpp.25.o: In function `Copter::init_csmag()':
    sensors.cpp:(.text._ZN6Copter10init_csmagEv+0x10): undefined reference to `RingBuffer<int>::_singleton'
    sensors.cpp:(.text._ZN6Copter10init_csmagEv+0x22): undefined reference to `RingBuffer<int>::print_info()'
    collect2: error: ld returned 1 exit status

    perhaps problem with singletons in template classes???
    */
    //RingBuffer<int32_t> *induction_value_buffer = RingBuffer<int32_t>::get_singleton();

    //RingBufferInt32 *induction_value_buffer = RingBufferInt32::get_singleton();
    induction_value_buffer = new RingBuffer<int32_t>(CSMAG_INDUCTION_VALUE_BUFFER_SIZE);

    if (ISDOBUFFERDEBUGPRINTOUTS || ISDOVERBOSEINITPRINTOUTS) {
        printf("(D10) induction_value_buffer after get_singleton():\n");
        induction_value_buffer->print_info();
    }
    
    //RingBufferUInt64 *induction_value_timestamp_buffer = RingBufferUInt64::get_singleton();
    // TODO: doublecheck, whether we need 64 or 32 bit values here
    induction_value_timestamp_buffer = new RingBuffer<uint64_t>(CSMAG_INDUCTION_VALUE_BUFFER_SIZE);

    if (ISDOBUFFERDEBUGPRINTOUTS || ISDOVERBOSEINITPRINTOUTS) {
        printf("(E10) induction_value_timestamp_buffer after get_singleton():\n");
        induction_value_timestamp_buffer->print_info();
    }

    // init UART 
    // TODO: properly use serial manager (cf. Rangefinder), doublecheck uart init
    // for now using https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL/examples/UART_test/UART_test.cpp

#if ISDOVERBOSEUARTCHECK
    AP_HAL::UARTDriver *uart0; // HERE
#endif
    // TODO: more elegant UART handling, if necessary?
    // uart_csmag_data = hal.uartC;            // use UART C (TELEM1 on Pixhawk1) for reading magnetometer data




    uart_csmag_data = UART_FOR_CSMAG_DATA;

    
    uart_csmag_data->begin(MAGNETOMETER_SERIAL_BAUDRATE);   
    
    
    
    // TODO: check configuration and baudrate, load it from configurations


    if (ISDOMAGDATAINITREADUARTCHECK) {
        hal.console->printf("1. inited UART: %p\n", uart_csmag_data);
        hal.console->printf("nullptr: %p\n", nullptr);
    }

    //uart_csmag_data->begin(MAGNETOMETER_SERIAL_BAUDRATE);

    // if (ISDOMAGDATAINITREADUARTCHECK) {
    //     hal.console->printf("2. inited UART: %p\n", uart_csmag_data);
    //     hal.console->printf("nullptr: %p\n", nullptr);
    // }

    //hal.scheduler->delay(1000); //Ensure that the uartA can be initialized
#if ISDOVERBOSEUARTCHECK
        //AP_HAL::UARTDriver *uart;
        //uart = hal.uartC;
        uart0 = uart_csmag_data;
        
        uart0->printf("Hello - this is TELEM1!\n");
        //hal.uartC->printf("Hello - this is TELEM1!\n");
#endif

    //hal.console->printf("2. HELLO - hal.console->printf ok - 190329T1338+0100\n");

   return true;
}

void Copter::read_csmag(void) {

    if (ISPRINTTIMESTAMPREADCSMAG) {
        hal.console->printf("Copter::read_csmag() at %" PRIu64 "\n", AP_HAL::micros64());
    }

    Csmag *_csmag;

    // uint32_t induction_value_maginterface_timestamp_i = 0;   // CHANGED HERE

    // TODO: declare as class variables
    // uint64_t induction_value_maginterface_timestamp_i = 0;
    // int32_t _induction_value_i = CSMAG_INVALID_INDUCTION_VALUE;
    //uint16_t count;
    int16_t nbytes; // = uart_csmag_data->available();
    // bool is_timestamp_mode;// = false;
    // bool is_induction_mode;// = false;
    // int values_remaining_bytes = 4;

    //induction_value_timestamp_i = induction_value_maginterface_timestamp_i;
    //induction_value_i = _induction_value_i;
    
    // begin of fake data generation

    if (IS_GENERATE_FAKE_CSMAG_STATE) {
        printf("WARNING!!! IS_GENERATE_FAKE_CSMAG_STATE is deprecated, use IS_GENERATE_FAKE_CSMAG_INDUCTION_VALUES instead\n");

        Csmag *_csmag = new Csmag();            // costs a lot of time TODO: remove this
        
        _csmag->csmag_state->time_usec = AP_HAL::micros64();
        //
        int i;
        for (i = 0; i < CSMAG_INDUCTION_ARRAY_SIZE; i++) {
            _csmag->csmag_state->induction[i] = i;              
        }
        _csmag->csmag_state->induction[0] = 42;
        _csmag->csmag_state->induction[1] = _csmag->csmag_state->time_usec / ( (typeof(_csmag->csmag_state->induction[1])) 1e6 ) ;        // =full seconds 
        _csmag->csmag_state->induction[2] = _csmag->csmag_state->time_usec % 100;        // last 2 digits of time_usec
        // some sine wave: periode T=2*pi s, amplitude y = +- 100 units
        _csmag->csmag_state->induction[3] = (typeof(_csmag->csmag_state->induction[3])) (100 * sinf(_csmag->csmag_state->time_usec /  1e6));   
    }

    if (IS_GENERATE_FAKE_CSMAG_INDUCTION_VALUES) {
        // generate and push fake timestamp (works)

        
        if (ISDOVERBOSEDEBUGPRINTOUTS) {
            // works
            hal.console->printf("induction_value_buffer.GetObjectCounter(): %d\n", 
                induction_value_buffer->GetObjectCounter());
            hal.console->printf("induction_value_timestamp_buffer.GetObjectCounter(): %d\n", 
                induction_value_timestamp_buffer->GetObjectCounter());
            hal.console->printf("---\n");
        }

        induction_value_maginterface_timestamp_i = AP_HAL::micros64();
        induction_value_timestamp_buffer->enqueue(induction_value_maginterface_timestamp_i);
        // 1 timestamp for <CSMAG_INDUCTION_ARRAY_SIZE> induction values, timestamp first
        //  WRONG! 1 timestamp for 1 induction value, check_send_csmag takes care of message
        // if (induction_value_buffer.GetObjectCounter() % CSMAG_INDUCTION_ARRAY_SIZE == 0) {
        //     if ( (induction_value_buffer.GetObjectCounter() / 10) 
        //     == (induction_value_timestamp_buffer.GetObjectCounter() - 1) ) {
        //         induction_value_maginterface_timestamp_i = AP_HAL::micros64();
        //         induction_value_timestamp_buffer.enqueue(induction_value_maginterface_timestamp_i);
        //     } else if ( (induction_value_buffer.GetObjectCounter() / 10) 
        //     == (induction_value_timestamp_buffer.GetObjectCounter() - 1) ) {
        //         // also ok, in "timestamp_mode" now
        //     } else {
        //         hal.console->printf("WARNING!!! Error generating fake data in Pixhawk1\n");
        //     }
        // }

        // generate and push fake induction values

        if (IS_GENERATE_FAKE_CSMAG_INDUCTION_VALUES_SIN) {
            _induction_value_i = (typeof(_induction_value_i)) (100 * sinf(induction_value_maginterface_timestamp_i /  1e6));   
        } else {
            _induction_value_i = 42;                     // generate constant output of 42
        }
        induction_value_buffer->enqueue(_induction_value_i);

        if (ISMONITORCHECKCSMAG) {
            printf("+");
            printf("%x", induction_value_buffer->GetObjectCounter());
            printf("%x", induction_value_timestamp_buffer->GetObjectCounter());
        }

        if (!CSMAG_IS_USE_INDUCTION_VALUE_BUFFER_MODE) {
            printf("WARNING! Please use CSMAG_IS_USE_INDUCTION_VALUE_BUFFER_MODE, using IS_GENERATE_FAKE_CSMAG_STATE is deprecated\n");
        }

        // end of fake data generation
    } else {
        // read real values from MAGInterface via UART

        if (ISDOMAGDATAREADUARTCHECK) {
            //printf("UART: %p\n", uart_csmag_data);
            hal.console->printf("UART: %p\n", uart_csmag_data);         // fixed it
            hal.console->printf("hal.uartC: %p\n", hal.uartC);         // 
        }

        if (uart_csmag_data != nullptr) {

            if (ISDOMAGDATAREADUARTCHECK) {
                printf("read from UART (as hexdump):\n", uart_csmag_data);
            }

            nbytes = uart_csmag_data->available();
            // must preserve modi, dont overwrite mode from last read_csmag() call!
            // is_timestamp_mode = false;
            // is_induction_mode = false;
            //int values_remaining_bytes = 4;
            uint8_t c;
            while (nbytes-- > 0) {
                //char c = uart_csmag_data->read();
                c = uart_csmag_data->read();        // TODO: check for -1

                if (ISDOMAGDATAREADUARTCHECK) {
                    printf("%02x ", c);
                }

                if (is_timestamp_mode) {
                    // read uint32_t timestamp
                    if (IS_MAG_INTERFACE_PROTOCOL_BIG_ENDIAN) {
                        induction_value_maginterface_timestamp_i <<= 8 * sizeof(c);
                        induction_value_maginterface_timestamp_i |= c;
                    } else {
                        // TODO: implement little endian
                        hal.console->printf("Error! Reading MAGInterface data in little endian is not implemented yet.\n");
                    }
                    values_remaining_bytes--;
                } else if (is_induction_mode) {
                    // read int32_t induction value
                    if (IS_MAG_INTERFACE_PROTOCOL_BIG_ENDIAN) {
                        _induction_value_i <<= 8 * sizeof(c);
                        _induction_value_i |= c;
                    } else {
                        // TODO: implement little endian
                        hal.console->printf("Error! Reading MAGInterface data in little endian is not implemented yet.\n");
                    }
                    values_remaining_bytes--;
                } else {
                    // default mode: check for state
                    switch (c) {
                    case 'T':
                        is_timestamp_mode = true;
                        is_induction_mode = false;
                        values_remaining_bytes = 4;                     // type uint32_t from MAGInterface, stored as 64 b
                        induction_value_maginterface_timestamp_i = 0;   // init value TODO: class variable!

                        if (IS_PRINT_MISC_CHAR_UART_BUFFER) {
                            if (rest_uart_buffer->GetObjectCounter() > 0) {
                                hal.console->printf("rest_uart_buffer as hex chars: [");
                                while (rest_uart_buffer->GetObjectCounter() > 0) {
                                    hal.console->printf("%02x ", rest_uart_buffer->dequeue());
                                }
                                hal.console->printf("]\n");
                            }
                        }
                        break;
                    case 'I':
                        is_timestamp_mode = false;
                        is_induction_mode = true;
                        values_remaining_bytes = 4;                     // type int32_t
                        _induction_value_i = 0;

                        if (IS_PRINT_MISC_CHAR_UART_BUFFER) {
                            if (rest_uart_buffer->GetObjectCounter() > 0) {
                                hal.console->printf("rest_uart_buffer as hex chars: [ ");
                                while (rest_uart_buffer->GetObjectCounter() > 0) {
                                    hal.console->printf("%02x ", rest_uart_buffer->dequeue());
                                }
                                hal.console->printf("]\n");
                            }
                        }
                        break;
                    default:
                        rest_uart_buffer->enqueue(c);
                    }
                }

                if (values_remaining_bytes <= 0) {
                    // number (timestamp or induction value) has been read

                    // push new timestamp or induction value
                    if (CSMAG_IS_USE_INDUCTION_VALUE_BUFFER_MODE) {

                        if (ISDOVERBOSEDEBUGPRINTOUTS) {
                            hal.console->printf("R");
                        }

                        // TODO: check if numbers of timestamps and induction values match

                        if (is_timestamp_mode) {
                            // use RingBuffer<T> instead of unnecessary RingBuffer singleton classes
                            // CONTINUE HERE
                            if (IS_DO_INDUCTION_VALUE_TIMEOUT_BUFFER_FLUSH) {
                                
                                if (ISDOFORMERTEMPVERBOSEDEBUG) {
                                    hal.console->printf("induction_value_timestamp_buffer->GetLastObject(): %" PRIu64 "\n",
                                     induction_value_timestamp_buffer->GetLastObject());
                                }

                                if ( (induction_value_timestamp_buffer->GetObjectCounter() > 0) && 
                                ( (induction_value_maginterface_timestamp_i - induction_value_timestamp_buffer->GetLastObject())\
                                > INDUCTION_VALUE_TIMEOUT_BUFFER_FLUSH_THRESHOLD ) ) {
                                    // if last value older than a certain threshold:
                                    //  delete all of the old data, because it might be compromised or deprecated
                                    if (ISDOTEMPVERBOSEDEBUG) {
                                        // hal.console->printf("last timestamp value received via UART is older than threshold.\n");
                                        hal.console->printf("last timestamp is %" PRIu64 "us old. Deleting old values.\n", 
                                            induction_value_maginterface_timestamp_i - induction_value_timestamp_buffer->GetLastObject());
                                        hal.console->printf("induction_value_maginterface_timestamp_i: %" PRIu64 "\n",
                                            induction_value_maginterface_timestamp_i);
                                        hal.console->printf("induction_value_timestamp_buffer->GetLastObject(): %" PRIu64 "\n",
                                            induction_value_timestamp_buffer->GetLastObject());
                                    }

                                    while (induction_value_timestamp_buffer->GetObjectCounter() > 0) {
                                        induction_value_timestamp_buffer->dequeue();
                                    }
                                    while (induction_value_buffer->GetObjectCounter() > 0) {
                                        induction_value_buffer->dequeue();
                                    }
                                }
                            }

                            if (ISDOMAGDATAREADUARTCHECK) {
                                // TODO: printout enqueued data in hex in right alignment (%016x???)
                                hal.console->printf("enqueue timestamp %" PRIu64 " == 0x%" PRIx64 "\n", 
                                    induction_value_maginterface_timestamp_i, induction_value_maginterface_timestamp_i);
                            }

                            induction_value_timestamp_buffer->enqueue(induction_value_maginterface_timestamp_i);
                        } else if (is_induction_mode) {

                            if (ISDOMAGDATAREADUARTCHECK) {
                                hal.console->printf("enqueue induction value %" PRId32 " == 0x%" PRIx32 "\n", 
                                    _induction_value_i, _induction_value_i);
                            }

                            induction_value_buffer->enqueue(_induction_value_i);
                        } else {
                            
                            if (ISDOMAGDATAREADUARTCHECK) {
                                hal.console->printf("!!! rest of timestamp: %" PRIu64 "\n", 
                                    induction_value_maginterface_timestamp_i);
                                hal.console->printf("rest induction value %" PRId32 "\n", _induction_value_i);
                            }

                            // invalid mode
                            // should only happen in the beginning
                            if (IS_PRINT_WARNING_TIMESTAMPS) {
                                hal.console->printf("At %" PRIu64 ": ", AP_HAL::micros64());
                            }
                            hal.console->printf("Warning! Copter::read_csmag() couldn't read data.\n");
                        }

                        if (ISMONITORCHECKCSMAG) {
                            printf("+");
                            printf("%x", induction_value_buffer->GetObjectCounter());
                            printf("%x", induction_value_timestamp_buffer->GetObjectCounter());
                        }

                    } else {
                        printf("WARNING! Please use CSMAG_IS_USE_INDUCTION_VALUE_BUFFER_MODE, using IS_GENERATE_FAKE_CSMAG_STATE is deprecated\n");
                    }

                    // go back to default mode after the according number of bytes has been read out
                    is_timestamp_mode = false;
                    is_induction_mode = false;
                }
            }
        } else {
            // TODO: some exception handling?
            #if ISPRINTOUTNOUARTCONNECTIONVERBOSE
                hal.console->printf("Error! Can't find UART.");
            #elif ISPRINTOUTNOUARTCONNECTIONSIMPLE
                hal.console->printf("X");
            #endif
            
        }

        if (ISDOMAGDATAREADUARTCHECK) {
            printf("\nEnd of UART data\n");
        }
    }
    

    // pushing the new status with 50 Hz only makes sense, if we generate fake csmag buffer states
    // normally we'd want to to this only in check_send_csmag
    if (CSMAG_IS_USE_BUFFER_MODE && IS_GENERATE_FAKE_CSMAG_STATE) {
        if (ISDOBUFFERDEBUGPRINTOUTS) {
            csmag_state_buffer.print_info();
            printf("push most recent state.\n");
        }
        
        csmag_state_buffer.push(_csmag->csmag_state);
        
        if (ISDOBUFFERDEBUGPRINTOUTS) {
            csmag_state_buffer.print_info();
            printf("after push---\n");
        }

        csmag = *_csmag;    // set Csmag singleton to most recent csmag (redundant in buffer mode)
    }

    // if necessary, send a customary message to gcs, to prove the new cusom arducopter code is running 
#if ISDOREPEATEDGCSMESSAGE
    time_since_start = AP_HAL::micros64();      // update time
    if (time_since_start - time_last_repeated_gcs_message > GCSMESSAGEINTERVAL)  {
        //gcs().send_text(MAV_SEVERITY_CRITICAL, "Hey ho :) %6.4f", (double)3.1416f);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Hals- und Beinbruch :)");
        time_last_repeated_gcs_message = time_since_start;

        if (ISDOMAGDATAINITREADUARTCHECK) {
            hal.console->printf("(M10) inited UART: %p\n", uart_csmag_data);
            hal.console->printf("(M20) nullptr: %p\n", nullptr);
        }

        if (ISDOINTERVALMAGDATAREADUARTCHECK) {
            // TODO: continue here
        }

        if (ISDOVERBOSEDEBUGPRINTOUTS) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Sending UART test messages on all UART ports");
            hal.uartA->printf("HELLO UART-A !!!\n");        // only A seems to work rightnow
            hal.uartB->printf("HELLO UART-B !!!\n");
            hal.uartC->printf("HELLO UART-C !!!\n");
            hal.uartD->printf("HELLO UART-D !!!\n");
            hal.uartE->printf("HELLO UART-E !!!\n");
        }

        if (ISDOVERBOSEUARTCHECK) {
            AP_HAL::UARTDriver *uart;
            int i;
            int repeat_uart_msg = 10;

            // // get's printed out in GCS
            // // cannot change flight mode
            // uart = hal.uartA;
            // uart->printf("Hello UARTA!!!\n");
            // for (i = 0; i < repeat_uart_msg; i++) {
            //     uart->printf("Hello! ");
            // }

            uart = hal.uartB;
            uart->printf("Hello UARTB!!!\n");
            for (i = 0; i < repeat_uart_msg; i++) {
                uart->printf("Hello! ");
            }
            // hal.console->printf works on mavproxy, using Pixhawk1
            //  mavproxy.py --map --master=/dev/ttyACM4
            hal.console->printf("UART B pointer: %p\n", uart);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "UART B pointer: %p", uart);

            uart = hal.uartC;
            uart->printf("Hello UARTC!!!\n");
            for (i = 0; i < repeat_uart_msg; i++) {
                uart->printf("Hello! ");
            }
            hal.console->printf("UART C pointer: %p\n", uart);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "UART C pointer: %p", uart);

            uart = hal.uartD;
            uart->printf("Hello UARTD!!!\n");
            for (i = 0; i < repeat_uart_msg; i++) {
                uart->printf("Hello! ");
            }
            hal.console->printf("UART D pointer: %p\n", uart);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "UART D pointer: %p", uart);

            uart = hal.uartE;
            uart->printf("Hello UARTE!!!\n");
            for (i = 0; i < repeat_uart_msg; i++) {
                uart->printf("Hello! ");
            }
            hal.console->printf("UART E pointer: %p\n", uart);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "UART E pointer: %p", uart);
            
            // hal.uartC->printf("Hello TELEM1!!!\n");
            // for (i = 0; i < 100; i++) {
            //     hal.uartC->printf("Hello! ");
            // }
        }
    }
#endif

}


// checks wether there are enough csmag induction values for a new message and tries to send it
void Copter::check_send_csmag() {

    uint32_t timestamp_comp_mag;            // comparatative timestamp from MAGInterface
    uint64_t timestamp_comp_pix;            // comparatative timestamp from Pixhawk
    int64_t _timestamp_comp_delta;           // Pixhawk time - MAGInterface time
    //Csmag *_csmag = new Csmag();    // TODO: remove this
    Csmag *_csmag = &csmag;
    if (CSMAG_IS_USE_INDUCTION_VALUE_BUFFER_MODE) {

        if (ISMONITORCHECKCSMAG) {
            printf("called Copter::check_send_csmag()\n");
            printf("induction_value_buffer.GetObjectCounter(): %d\n", induction_value_buffer->GetObjectCounter());
            printf("induction_value_timestamp_buffer.GetObjectCounter(): %d\n", induction_value_timestamp_buffer->GetObjectCounter());
        }

        // check if there are enough induction values for a new message
        while (induction_value_buffer->GetObjectCounter() >= CSMAG_INDUCTION_ARRAY_SIZE) {
            // if yes: form a message 
            int i;
            _csmag->csmag_state->time_usec = induction_value_timestamp_buffer->dequeue();    // use timestamp of first induction value
            _csmag->csmag_state->induction[0] = induction_value_buffer->dequeue();
            for (i = 1; i < CSMAG_INDUCTION_ARRAY_SIZE; i++) {
                _csmag->csmag_state->induction[i] = induction_value_buffer->dequeue();       // fill induction array with induction values
                induction_value_timestamp_buffer->dequeue();                                 // throw away other timestamps
                // we might check here if the samplerate is correct, before throwing the timestamps away
            }


            // synchronize timestamps
            //CSMAG_TIMESTAMP_SYNCHRONIZATION_TRIGGER_DIFFERENCE
            // if (is_first_csmag_message) {
            //     // only synchronize in first message
            //     // TODO: think about when/how often to synchronize
            //     // TODO: synchronize it in interval?
            //     timestamp_comp_mag = _csmag->csmag_state->time_usec;                            // first timestamp of this future message from MAGClient
            //     timestamp_comp_pix = AP_HAL::micros64();
            //     timestamp_comp_delta = timestamp_comp_pix - timestamp_comp_mag;
            //     is_first_csmag_message = false;d
            // }
            timestamp_comp_mag = _csmag->csmag_state->time_usec;                            // first timestamp of this future message from MAGClient
            timestamp_comp_pix = AP_HAL::micros64();
            _timestamp_comp_delta = timestamp_comp_pix - timestamp_comp_mag;

            if (is_first_csmag_message || ( (_timestamp_comp_delta - timestamp_comp_delta) >  CSMAG_TIMESTAMP_SYNCHRONIZATION_TRIGGER_DIFFERENCE) ) {
                // if first message OR timestamp_comp_delta changed more than the threshold:
                timestamp_comp_delta = _timestamp_comp_delta;   // synchronize again
                is_first_csmag_message = false;
            }

            // use Pixhawk timestamp instead of MAGInterface timestamp
            _csmag->csmag_state->time_usec += timestamp_comp_delta;

            if (CSMAG_IS_USE_BUFFER_MODE) {
                csmag_state_buffer.push(_csmag->csmag_state);
            }
            csmag = *_csmag; // set Csmag singleton to most recent csmag (redundant in buffer mode)

            // and send the message
            gcs().send_message(MSG_CSMAG0);

            if (ISMONITORCHECKCSMAG) {
                printf("send CSMAG0 message\n");
            }

            // to find out how many messages got sent per check_send_csmag() call (all printed in 1 line)
            if (ISPRINTOUTSENDCSMAG) {
                hal.console->printf("CSMAG ");
            }
        }
        if (ISPRINTOUTSENDCSMAG) {
            hal.console->printf("\n");
        }
    } else {
        printf("Warning! Use CSMAG_IS_USE_INDUCTION_VALUE_BUFFER_MODE! Non buffer mode is deprecated.\n");
        if (CSMAG_IS_USE_BUFFER_MODE) {
                csmag_state_buffer.push(_csmag->csmag_state);
            }
            csmag = *_csmag; // set Csmag singleton to most recent csmag (redundant in buffer mode)

            // and send the message
            gcs().send_message(MSG_CSMAG0);
    }
}


// }

/*
  update RPM sensors
 */
void Copter::rpm_update(void)
{
#if RPM_ENABLED == ENABLED
    rpm_sensor.update();
    if (rpm_sensor.enabled(0) || rpm_sensor.enabled(1)) {
        if (should_log(MASK_LOG_RCIN)) {
            DataFlash.Log_Write_RPM(rpm_sensor);
        }
    }
#endif
}

// initialise compass
void Copter::init_compass()
{
    if (!g.compass_enabled) {
        return;
    }

    if (!compass.init() || !compass.read()) {
        // make sure we don't pass a broken compass to DCM
        hal.console->printf("COMPASS INIT ERROR\n");
        Log_Write_Error(ERROR_SUBSYSTEM_COMPASS,ERROR_CODE_FAILED_TO_INITIALISE);
        return;
    }
    ahrs.set_compass(&compass);
}

/*
  if the compass is enabled then try to accumulate a reading
  also update initial location used for declination
 */
void Copter::compass_accumulate(void)
{
    if (!g.compass_enabled) {
        return;
    }

    compass.accumulate();

    // update initial location used for declination
    if (!ap.compass_init_location) {
        Location loc;
        if (ahrs.get_position(loc)) {
            compass.set_initial_location(loc.lat, loc.lng);
            ap.compass_init_location = true;
        }
    }
}

// initialise optical flow sensor
void Copter::init_optflow()
{
#if OPTFLOW == ENABLED
    // initialise optical flow sensor
    optflow.init();
#endif      // OPTFLOW == ENABLED
}

// called at 200hz
#if OPTFLOW == ENABLED
void Copter::update_optical_flow(void)
{
    static uint32_t last_of_update = 0;

    // exit immediately if not enabled
    if (!optflow.enabled()) {
        return;
    }

    // read from sensor
    optflow.update();

    // write to log and send to EKF if new data has arrived
    if (optflow.last_update() != last_of_update) {
        last_of_update = optflow.last_update();
        uint8_t flowQuality = optflow.quality();
        Vector2f flowRate = optflow.flowRate();
        Vector2f bodyRate = optflow.bodyRate();
        const Vector3f &posOffset = optflow.get_pos_offset();
        ahrs.writeOptFlowMeas(flowQuality, flowRate, bodyRate, last_of_update, posOffset);
        if (g.log_bitmask & MASK_LOG_OPTFLOW) {
            Log_Write_Optflow();
        }
    }
}
#endif  // OPTFLOW == ENABLED

void Copter::compass_cal_update()
{
    static uint32_t compass_cal_stick_gesture_begin = 0;

    if (!hal.util->get_soft_armed()) {
        compass.compass_cal_update();
    }

    if (compass.is_calibrating()) {
        if (channel_yaw->get_control_in() < -4000 && channel_throttle->get_control_in() > 900) {
            compass.cancel_calibration_all();
        }
    } else {
        bool stick_gesture_detected = compass_cal_stick_gesture_begin != 0 && !motors->armed() && channel_yaw->get_control_in() > 4000 && channel_throttle->get_control_in() > 900;
        uint32_t tnow = millis();

        if (!stick_gesture_detected) {
            compass_cal_stick_gesture_begin = tnow;
        } else if (tnow-compass_cal_stick_gesture_begin > 1000*COMPASS_CAL_STICK_GESTURE_TIME) {
#ifdef CAL_ALWAYS_REBOOT
            compass.start_calibration_all(true,true,COMPASS_CAL_STICK_DELAY,true);
#else
            compass.start_calibration_all(true,true,COMPASS_CAL_STICK_DELAY,false);
#endif
        }
    }
}

void Copter::accel_cal_update()
{
    if (hal.util->get_soft_armed()) {
        return;
    }
    ins.acal_update();
    // check if new trim values, and set them
    float trim_roll, trim_pitch;
    if(ins.get_new_trim(trim_roll, trim_pitch)) {
        ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
    }

#ifdef CAL_ALWAYS_REBOOT
    if (ins.accel_cal_requires_reboot()) {
        hal.scheduler->delay(1000);
        hal.scheduler->reboot(false);
    }
#endif
}

// initialise proximity sensor
void Copter::init_proximity(void)
{
#if PROXIMITY_ENABLED == ENABLED
    g2.proximity.init();
    g2.proximity.set_rangefinder(&rangefinder);
#endif
}

// update error mask of sensors and subsystems. The mask
// uses the MAV_SYS_STATUS_* values from mavlink. If a bit is set
// then it indicates that the sensor or subsystem is present but
// not functioning correctly.
void Copter::update_sensor_status_flags(void)
{
    // default sensors present
    control_sensors_present = MAVLINK_SENSOR_PRESENT_DEFAULT;

    // first what sensors/controllers we have
    if (g.compass_enabled) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_MAG; // compass present
    }
    if (gps.status() > AP_GPS::NO_GPS) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_GPS;
    }
#if OPTFLOW == ENABLED
    if (optflow.enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
    }
#endif
#if PRECISION_LANDING == ENABLED
    if (precland.enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    }
#endif
#if VISUAL_ODOMETRY_ENABLED == ENABLED
    if (g2.visual_odom.enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    }
#endif
    if (ap.rc_receiver_present) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    }
    if (copter.DataFlash.logging_present()) { // primary logging only (usually File)
        control_sensors_present |= MAV_SYS_STATUS_LOGGING;
    }
#if PROXIMITY_ENABLED == ENABLED
    if (copter.g2.proximity.sensor_present()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_PROXIMITY;
    }
#endif
#if AC_FENCE == ENABLED
    if (copter.fence.sys_status_present()) {
        control_sensors_present |= MAV_SYS_STATUS_GEOFENCE;
    }
#endif
#if RANGEFINDER_ENABLED == ENABLED
    if (rangefinder.has_orientation(ROTATION_PITCH_270)) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
    }
#endif

    // all sensors are present except these, which may be set as enabled below:
    control_sensors_enabled = control_sensors_present & (~MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL &
                                                         ~MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL &
                                                         ~MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS &
                                                         ~MAV_SYS_STATUS_LOGGING &
                                                         ~MAV_SYS_STATUS_SENSOR_BATTERY &
                                                         ~MAV_SYS_STATUS_GEOFENCE &
                                                         ~MAV_SYS_STATUS_SENSOR_LASER_POSITION &
                                                         ~MAV_SYS_STATUS_SENSOR_PROXIMITY);

    switch (control_mode) {
    case AUTO:
    case AVOID_ADSB:
    case GUIDED:
    case LOITER:
    case RTL:
    case CIRCLE:
    case LAND:
    case POSHOLD:
    case BRAKE:
    case THROW:
    case SMART_RTL:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        break;
    case ALT_HOLD:
    case GUIDED_NOGPS:
    case SPORT:
    case AUTOTUNE:
    case FLOWHOLD:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        break;
    default:
        // stabilize, acro, drift, and flip have no automatic x,y or z control (i.e. all manual)
        break;
    }

    // set motors outputs as enabled if safety switch is not disarmed (i.e. either NONE or ARMED)
    if (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
    }

    if (copter.DataFlash.logging_enabled()) {
        control_sensors_enabled |= MAV_SYS_STATUS_LOGGING;
    }

    if (battery.num_instances() > 0) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_BATTERY;
    }
#if AC_FENCE == ENABLED
    if (copter.fence.sys_status_enabled()) {
        control_sensors_enabled |= MAV_SYS_STATUS_GEOFENCE;
    }
#endif
#if PROXIMITY_ENABLED == ENABLED
    if (copter.g2.proximity.sensor_enabled()) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_PROXIMITY;
    }
#endif


    // default to all healthy
    control_sensors_health = control_sensors_present;

    if (!barometer.all_healthy()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    }
    if (!g.compass_enabled || !compass.healthy() || !ahrs.use_compass()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_3D_MAG;
    }
    if (!gps.is_healthy()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_GPS;
    }
    if (!ap.rc_receiver_present || failsafe.radio) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    }
#if OPTFLOW == ENABLED
    if (!optflow.healthy()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
    }
#endif
#if PRECISION_LANDING == ENABLED
    if (precland.enabled() && !precland.healthy()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    }
#endif
#if VISUAL_ODOMETRY_ENABLED == ENABLED
    if (g2.visual_odom.enabled() && !g2.visual_odom.healthy()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    }
#endif
    if (!ins.get_gyro_health_all() || !ins.gyro_calibrated_ok_all()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_3D_GYRO;
    }
    if (!ins.get_accel_health_all()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_3D_ACCEL;
    }

    if (ahrs.initialised() && !ahrs.healthy()) {
        // AHRS subsystem is unhealthy
        control_sensors_health &= ~MAV_SYS_STATUS_AHRS;
    }

    if (copter.DataFlash.logging_failed()) {
        control_sensors_health &= ~MAV_SYS_STATUS_LOGGING;
    }

#if PROXIMITY_ENABLED == ENABLED
    if (copter.g2.proximity.sensor_failed()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_PROXIMITY;
    }
#endif

#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    switch (terrain.status()) {
    case AP_Terrain::TerrainStatusDisabled:
        break;
    case AP_Terrain::TerrainStatusUnhealthy:
        // To-Do: restore unhealthy terrain status reporting once terrain is used in copter
        //control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        //control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        //break;
    case AP_Terrain::TerrainStatusOK:
        control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_health  |= MAV_SYS_STATUS_TERRAIN;
        break;
    }
#endif

#if RANGEFINDER_ENABLED == ENABLED
    if (rangefinder_state.enabled) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        if (!rangefinder.has_data_orient(ROTATION_PITCH_270)) {
            control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        }
    }
#endif

    if (!ap.initialised || ins.calibrating()) {
        // while initialising the gyros and accels are not enabled
        control_sensors_enabled &= ~(MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL);
        control_sensors_health &= ~(MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL);
    }

    if (!copter.battery.healthy() || copter.battery.has_failsafed()) {
         control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_BATTERY;
    }
#if AC_FENCE == ENABLED
    if (copter.fence.sys_status_failed()) {
        control_sensors_health &= ~MAV_SYS_STATUS_GEOFENCE;
    }
#endif

#if FRSKY_TELEM_ENABLED == ENABLED
    // give mask of error flags to Frsky_Telemetry
    frsky_telemetry.update_sensor_status_flags(~control_sensors_health & control_sensors_enabled & control_sensors_present);
#endif
}

// init visual odometry sensor
void Copter::init_visual_odom()
{
#if VISUAL_ODOMETRY_ENABLED == ENABLED
    g2.visual_odom.init();
#endif
}

// update visual odometry sensor
void Copter::update_visual_odom()
{
#if VISUAL_ODOMETRY_ENABLED == ENABLED
    // check for updates
    if (g2.visual_odom.enabled() && (g2.visual_odom.get_last_update_ms() != visual_odom_last_update_ms)) {
        visual_odom_last_update_ms = g2.visual_odom.get_last_update_ms();
        float time_delta_sec = g2.visual_odom.get_time_delta_usec() / 1000000.0f;
        ahrs.writeBodyFrameOdom(g2.visual_odom.get_confidence(),
                                g2.visual_odom.get_position_delta(),
                                g2.visual_odom.get_angle_delta(),
                                time_delta_sec,
                                visual_odom_last_update_ms,
                                g2.visual_odom.get_pos_offset());
        // log sensor data
        DataFlash.Log_Write_VisualOdom(time_delta_sec,
                                       g2.visual_odom.get_angle_delta(),
                                       g2.visual_odom.get_position_delta(),
                                       g2.visual_odom.get_confidence());
    }
#endif
}

// winch and wheel encoder initialisation
void Copter::winch_init()
{
#if WINCH_ENABLED == ENABLED
    g2.wheel_encoder.init();
    g2.winch.init(&g2.wheel_encoder);
#endif
}

// winch and wheel encoder update
void Copter::winch_update()
{
#if WINCH_ENABLED == ENABLED
    g2.wheel_encoder.update();
    g2.winch.update();
#endif
}
