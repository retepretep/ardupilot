#include "Csmag.h"

// added by peter
// TODO: put into Csmag.cpp and make sure csmag files are added to compile path

Csmag::Csmag() {

    csmag_state = new CsmagState();
    csmag_state->time_usec = 0;
    int i;
    for (i = 0; i < CSMAG_INDUCTION_ARRAY_SIZE; i++) {
        csmag_state->induction[i] = CSMAG_INVALID_INDUCTION_VALUE;
    }

    // initing buffers here or in Copter::init_csmag() ?
    induction_value_buffer = new RingBuffer<int32_t>(CSMAG_INDUCTION_VALUE_BUFFER_SIZE);
    induction_value_timestamp_buffer = new RingBuffer<uint64_t>(CSMAG_INDUCTION_VALUE_BUFFER_SIZE);

    uart = UART_FOR_CSMAG_DATA;

    _singleton = this;

    // if (ISDOVERBOSEINITPRINTOUTS) {
    //     printf("called Csmag::Csmag()\n");          // gets called in the very beginning (probably when this class is included)
    // }
}

Csmag *Csmag::_singleton;


//bool Csmag::update(void) {
int Csmag::update(void) {    
    int counter_read_data = 0;
    uint64_t induction_value_timestamp_i = 0;
    int32_t induction_value_i = CSMAG_INVALID_INDUCTION_VALUE;
    int16_t nbytes_i = -1;
    // int n_uart_bytes_left = -1;
    bool is_successfully_read_data = false;

    // is_successfully_read_data = Csmag::get_reading(induction_value_timestamp_i, induction_value_i);
    // // TODO: while? and parse MAG data in UART buffer line by line? (FIFO?)
    // if (is_successfully_read_data) {
    //     // checking data probably not necessary here
    //     induction_value_timestamp_buffer->enqueue(induction_value_timestamp_i);
    //     induction_value_buffer->enqueue(induction_value_i);
    // }

    // while (is_successfully_read_data = Csmag::get_reading(induction_value_timestamp_i, induction_value_i)) {
    //     induction_value_timestamp_buffer->enqueue(induction_value_timestamp_i);
    //     induction_value_buffer->enqueue(induction_value_i);
    // }

    if (ISDOCSMAGUPDATEDEBUG) {
        hal.console->printf("(Up 10) call Csmag::update()\n");
    }
    
    // variante using the weird for loop, might have another behaviour than expected, but more compact

    // read mag values from uart, as long as there is data in Pixhawk's UART buffer
    // for (n_uart_bytes_left = Csmag::get_reading(induction_value_timestamp_i, induction_value_i);
    //         n_uart_bytes_left > 0;
    //         n_uart_bytes_left = Csmag::get_reading(induction_value_timestamp_i, induction_value_i)
    // ) {
    //     if (ISDOCSMAGUPDATEDEBUG) {
    //         hal.console->printf("read, ");
    //     }
    //     induction_value_timestamp_buffer->enqueue(induction_value_timestamp_i);
    //     induction_value_buffer->enqueue(induction_value_i);
    //     is_successfully_read_data = true;
    // }

    // read mag values from uart, as long as there is data in Pixhawk's UART buffer
    // TODO: CONTINUE HERE

    for (is_successfully_read_data = Csmag::get_reading(induction_value_timestamp_i, induction_value_i, nbytes_i);
            //n_uart_bytes_left > 0; // old
            //n_uart_bytes_left >= MAG_INTERFACE_V30_MESSAGE_SIZE;
            is_successfully_read_data || (nbytes_i > 0);
            is_successfully_read_data = Csmag::get_reading(induction_value_timestamp_i, induction_value_i, nbytes_i)
    ) {
        if (ISDOCSMAGUPDATEDEBUG || ISDOTEMPVERBOSEDEBUG) {
            hal.console->printf("read, ");
        }

        if (is_successfully_read_data) {
            #if ISDOTEMPVERBOSEDEBUG 
                hal.console->printf("induction_value_timestamp_buffer->enqueue(%" PRIu64 ");\n", induction_value_timestamp_i);
                hal.console->printf("induction_value_buffer->enqueue(%" PRId32 ");\n", induction_value_i);
            #endif

            induction_value_timestamp_buffer->enqueue(induction_value_timestamp_i);
            induction_value_buffer->enqueue(induction_value_i);
            counter_read_data++;
        }        
    }

    // variante: clearer and less compact

    // n_uart_bytes_left = Csmag::get_reading(induction_value_timestamp_i, induction_value_i);
    // while (n_uart_bytes_left > 0) {
    //     if (ISDOCSMAGUPDATEDEBUG) {
    //         hal.console->printf("read, ");
    //     }
    //     induction_value_timestamp_buffer->enqueue(induction_value_timestamp_i);
    //     induction_value_buffer->enqueue(induction_value_i);
    //     is_successfully_read_data = true;
    //     //
    //     n_uart_bytes_left = Csmag::get_reading(induction_value_timestamp_i, induction_value_i);
    // }

    // 

    if (ISDOCSMAGUPDATEDEBUG) {
        hal.console->printf("(Up 20) \n----\n");
    }

    // old return value of get_reading
    // while (is_successfully_read_data = Csmag::get_reading(induction_value_timestamp_i, induction_value_i)) {
    //     induction_value_timestamp_buffer->enqueue(induction_value_timestamp_i);
    //     induction_value_buffer->enqueue(induction_value_i);
    // }
    // perhaps some status update here, unnecessary for now
    return counter_read_data;
}

bool Csmag::get_reading(uint64_t &induction_timestamp_i, int32_t &induction_value_i, int16_t &nbytes) {
// int Csmag::get_reading(uint64_t &induction_timestamp_i, int32_t &induction_value_i) {

    bool is_successfully_read_data = false;

    if (uart == nullptr) {
        #if ISPRINTOUTNOUARTCONNECTIONVERBOSE
            hal.console->printf("Error! Can't find UART.\n");
        #elif ISPRINTOUTNOUARTCONNECTIONSIMPLE
            hal.console->printf("X");
        #endif

        return false;
        // return 0;   // no uart ==> no data available
    }
    
    // old variables from benewake
    // float sum_cm = 0;
    // uint16_t count = 0;
    // uint16_t count_out_of_range = 0;

    if (ISDOMAGDATAREADUARTCHECK) {
        hal.console->printf("(U005) read from UART %p (as hexdump):\n", uart);
    }

    // read any available lines from the lidar
    //int16_t nbytes = uart->available();
    nbytes = uart->available();
    while (nbytes-- > 0) {
        int16_t r = uart->read();       // try to read 1 byte from UART

        if (ISDOMAGDATAREADUARTCHECK) {
            if (r == -1) {
                hal.console->printf("(-1) ==> no available data at UART!\n");
            }
        }

        if (r < 0) {                    // read() returned (int16_t) -1 ==> nothing available
            continue;
        }

        uint8_t c = (uint8_t)r;         // otherwise read() returned uint8_t data byte

        if (ISDOMAGDATAREADUARTCHECK) {
            hal.console->printf("0x%02x ", c);
        }

        if (ISDOTEMPVERBOSEDEBUG) {
            hal.console->printf(" (U007) linebuf_len: %d\n", linebuf_len);
        }

        // if buffer is empty and this byte is <magic number>, add to buffer
        if (linebuf_len == 0) {
            if (ISDOTEMPVERBOSEDEBUG) {
                hal.console->printf("(U010) interpreting byte from UART: 0x%02x\n", c);
            }

            if (c == MAG_INTERFACE_V30_MAGIC_NUMBER1) {
                if (ISDOTEMPVERBOSEDEBUG) {
                    hal.console->printf("(U015) matches magic number1\n");
                }
                linebuf[linebuf_len++] = c;
            } else {
                // TODO: invalid MAGInterface message! clear buffer and return false here?
                // incorrect data
                #if ISPRINTOUTNOUARTCONNECTIONVERBOSE
                    hal.console->printf("Error!!! Can't interpret MAGInterface data 0x%02x == '%c'. linebuf_len: %d\n", 
                        c, c, (int) linebuf_len);
                #elif ISPRINTOUTNOUARTCONNECTIONSIMPLE
                    hal.console->printf("?");
                #endif

                if (ISDOTEMPVERBOSEDEBUG) {
                    //hal.console->printf("(U030) interpreting byte from UART: 0x%02x\n", c);

                }
            }
        // } else if (linebuf_len == 1) {
        //     // if buffer has 1 element and this byte is 0x59, add it to buffer
        //     // if not clear the buffer
        //     // TODO:find out why this happens???
        //     if (c == MAG_INTERFACE_V30_MAGIC_NUMBER1) {
        //         linebuf[linebuf_len++] = c;
        //     } else {
        //         linebuf_len = 0;
        //     }
        } else if (linebuf_len == 1) {
            if (ISDOTEMPVERBOSEDEBUG) {
                hal.console->printf("(U020) interpreting byte from UART: 0x%02x\n", c);
            }

            if (c == MAG_INTERFACE_V30_MAGIC_NUMBER0) {
                linebuf[linebuf_len++] = c;
            } else {
                // incorrect data
                #if ISPRINTOUTNOUARTCONNECTIONVERBOSE
                    hal.console->printf("Error!!! Can't interpret MAGInterface data 0x%02x == '%c'. linebuf_len: %d\n", 
                        c, c, (int) linebuf_len);
                #elif ISPRINTOUTNOUARTCONNECTIONSIMPLE
                    hal.console->printf("?");
                #endif

                // TODO: invalid MAGInterface message! clear buffer and return false here?
                if (ISDOTEMPVERBOSEDEBUG) {
                    hal.console->printf("(U030) Reset linebuf_len, return\n");
                }
                linebuf_len = 0;
                // return nbytes;
                return is_successfully_read_data;
            }
        } else {
            // add character to buffer
            linebuf[linebuf_len++] = c;

            if (ISDOTEMPVERBOSEDEBUG) {
                hal.console->printf("(U040) add char to linebuf from UART: 0x%02x\n", c);
            }

            // if buffer now has a full MAGInterface message: try to decode it
            if (linebuf_len == MAG_INTERFACE_V30_MESSAGE_SIZE) {

                if (ISDOTEMPVERBOSEDEBUG) {
                    //hal.console->printf("(U050) linebuf full, interpreting last byte from UART: 0x%02x\n", c);
                    hal.console->printf("(U050) linebuf full\n");
                    hal.console->printf("(U060) checking checksum\n");
                }

                // calculate checksum
                uint8_t checksum = 0;
                // exclude transmitted checksum from checksum calculation
                for (uint8_t i=0; i < (MAG_INTERFACE_V30_MESSAGE_SIZE - 1); i++) {    
                    checksum += linebuf[i];
                }
                // if checksum matches extract contents
                if (checksum == linebuf[MAG_INTERFACE_V30_MESSAGE_SIZE - 1]) {

                    if (ISDOTEMPVERBOSEDEBUG) {
                        hal.console->printf("(U070) checksums match. checksum: %d\n", checksum);
                    }

                    int i;
                    // read timestamp
                    induction_timestamp_i = 0;
                    for (i = 0; i < MAG_INTERFACE_SIZE_INDUCTION_VALUE_TIMESTAMP_I; i++) {
                        induction_timestamp_i |= (uint64_t) linebuf[\
                        MAG_INTERFACE_POS_START_INDUCTION_VALUE_TIMESTAMP_I\
                         + (MAG_INTERFACE_SIZE_INDUCTION_VALUE_TIMESTAMP_I - 1 - i)] << (8 * i);
                    }   // hopefully compiler uses loopunrolling here
                    // TODO: perhaps count out of range? for now we won't handle it here, but on GCS
                    // mag values outside of range are possible due to low pass filter
                    // uint16_t dist = ((uint16_t)linebuf[3] << 8) | linebuf[2];

                    // read induction values
                    induction_value_i = 0;
                    for (i = 0; i < MAG_INTERFACE_SIZE_INDUCTION_VALUE_I; i++) {
                        induction_value_i |= (int32_t) linebuf[\
                        MAG_INTERFACE_POS_START_INDUCTION_VALUE_I\
                         + (MAG_INTERFACE_SIZE_INDUCTION_VALUE_I - 1 - i)] << (8 * i);
                    }
                    is_successfully_read_data = true;
                    // only read 1 line/MAGinterface message for each function call of get_reading()
                    // return is_successfully_read_data;
                    if (ISDOTEMPVERBOSEDEBUG) {
                        hal.console->printf("(U080) return, nbytes: %d, is_successfully_read_data: %d\n",
                            nbytes, is_successfully_read_data);
                    }       
                    // added clear buffer to prevent from buffer overflow 
                    // clear buffer
                    linebuf_len = 0;

                    // return nbytes;
                    return is_successfully_read_data;
                } else {
                    // incorrect checksum
                    #if ISPRINTOUTNOUARTCONNECTIONVERBOSE
                        hal.console->printf("Error!!! Incorrect checksum for MAGinterface data.\n");
                    #elif ISPRINTOUTNOUARTCONNECTIONSIMPLE
                        hal.console->printf("C");
                    #endif
                }
                // clear buffer
                linebuf_len = 0;
            }
        }
    }

    if (ISDOMAGDATAREADUARTCHECK) {
        hal.console->printf("\n(U200) nbytes: %d\n--\n", (int) nbytes);
    }

    // if (count > 0) {
    //     // return average distance of readings
    //     reading_cm = sum_cm / count;
    //     return true;
    // }

    // if (count_out_of_range > 0) {
    //     // if only out of range readings return larger of
    //     // driver defined maximum range for the model and user defined max range + 1m
    //     float model_dist_max_cm = (model_type == BENEWAKE_TFmini) ? BENEWAKE_TFMINI_OUT_OF_RANGE_CM : BENEWAKE_TF02_OUT_OF_RANGE_CM;
    //     reading_cm = MAX(model_dist_max_cm, max_distance_cm() + BENEWAKE_OUT_OF_RANGE_ADD_CM);
    //     return true;
    // }

    // no readings so return false
    return is_successfully_read_data;
    // return nbytes;
}


//bool Csmag::synch_timestamp(uint64_t timestamp) {
    
void Csmag::synch_timestamp(uint64_t timestamp) {
    // bool is_synch_successfully = true;
    // // TODO: check if synch has been successfull

    // build message in linebuf
    char linebuf[MAG_INTERFACE_V30_MESSAGE_SIZE];
    //timestamp_micros = micros();                            // get us timestamp
    int i;
    int checksum;
    linebuf[ 0] = MAG_INTERFACE_V30_MAGIC_NUMBER1;          // byte 1 of magic number (not of linebuf)
    linebuf[ 1] = MAG_INTERFACE_V30_MAGIC_NUMBER0;
    
    linebuf[ 2] = MAG_INTERFACE_V30_MODE_TIMESTAMP_SYNCH;
    
    linebuf[ 3] = GET_BYTE_FROM_NUMBER(timestamp, 7);       // higher uint32_t 0 if timestamp is uint32_t
    linebuf[ 4] = GET_BYTE_FROM_NUMBER(timestamp, 6);
    linebuf[ 5] = GET_BYTE_FROM_NUMBER(timestamp, 5);
    linebuf[ 6] = GET_BYTE_FROM_NUMBER(timestamp, 4);
    
    linebuf[ 7] = GET_BYTE_FROM_NUMBER(timestamp, 3);
    linebuf[ 8] = GET_BYTE_FROM_NUMBER(timestamp, 2);
    linebuf[ 9] = GET_BYTE_FROM_NUMBER(timestamp, 1);
    linebuf[10] = GET_BYTE_FROM_NUMBER(timestamp, 0);

    linebuf[11] = 0;
    linebuf[12] = 0;
    linebuf[13] = 0;
    linebuf[14] = 0;

    for (i = 0, checksum = 0; i < MAG_INTERFACE_V30_MESSAGE_SIZE-1; i++) { // exclude checksum's slot itself
        checksum += linebuf[i];
    }
    linebuf[15] = (uint8_t) (checksum & 0xFF);

    // send message via serial port
    //Serial.print(linebuf);      // problem: 0-terminated strings
    for (i = 0; i < MAG_INTERFACE_V30_MESSAGE_SIZE; i++) {
        uart->printf("%c", linebuf[i]);
        // uart->printf(&(linebuf[i]));
        //Serial.write(linebuf[i]);
    }
}



#if IS_USE_CSMAGSTATEBUFFER

CsmagStateBuffer::CsmagStateBuffer() {

    buf = new Csmag::CsmagState*[CSMAG_BUFFER_SIZE];
    int i;
    for (i = 0; i < CSMAG_BUFFER_SIZE; i++) {
        buf[i] = nullptr;
    }
    object_counter = 0;
    first_index = INVALID_INDEX;
    next_index = 0;
    // set all available buffer slots as free
    is_free_mask = (1 << CSMAG_BUFFER_SIZE) - 1;    // get CSMAG_BUFFER_SIZE bits, rest is 0 (not free)

    // ..

    _singleton = this;
}

// push object to the end of ring queue buffer
// return true if pushing went successfully
bool CsmagStateBuffer::push(Csmag::CsmagState *new_obj) {
    //bool ret = false;
    if (is_full()) {
        // TODO: overwrite first object
        printf("ERROR! CsmagStateBuffer buffer overflow\n");
        //throw "CsmagStateBuffer buffer overflow";   // throw disabled
        return false;
    }
    // TODO: perhaps check if it is actually free (in mask)
    buf[next_index] = new_obj;
    if (object_counter == 0) {
        first_index = next_index;   // the new first index, if no object has been stored before
    }
    mark_occupied(next_index);
    //next_index++;
    next_index = (next_index + 1) % CSMAG_BUFFER_SIZE;
    object_counter++;
    //
    return true;
}

// pop object at relativeIndex (counting from first_index with 0), defaultly pop first object
Csmag::CsmagState* CsmagStateBuffer::pop(int relative_index) {
    //throw "This is not implemented yet";
    printf("ERROR! This is not implemented yet\n");
    return nullptr;
    // TODO: implement defragmentation if objects from the middle are popped

    int absolute_index = (first_index + relative_index) % CSMAG_BUFFER_SIZE;
    // check is there is actually an object
    if (is_free(absolute_index)) {
        //throw "CsmagStateBuffer does not contain an object at the given relative_index";
        printf("ERROR! CsmagStateBuffer does not contain an object at the given relative_index\n");
    }
    //
    mark_free(absolute_index);
    //first_index++;
    first_index = (first_index + 1) % CSMAG_BUFFER_SIZE;
    object_counter--;
    return buf[absolute_index];
}

// pop first object at (relative_index 0, counting from first_index with 0)
Csmag::CsmagState* CsmagStateBuffer::pop() {
    int relative_index = 0;                     
    int absolute_index = (first_index + relative_index) % CSMAG_BUFFER_SIZE;
    // check is there is actually an object
    if (is_free(absolute_index)) {
        //throw "CsmagStateBuffer does not contain an object at the given relative_index";
        printf("ERROR! CsmagStateBuffer does not contain an object at the given relative_index\n");
    }
    //
    mark_free(absolute_index);
    //first_index++;
    first_index = (first_index + 1) % CSMAG_BUFFER_SIZE;
    object_counter--;
    return buf[absolute_index];
}

// print n as binary digits on console
void CsmagStateBuffer::print_bits(uint32_t n) {
    uint32_t p;
    for (p = 1 << (8 * sizeof(n) - 1); p > 0; p >>= 1) {
        printf("%d", (bool) (n & p));
    }
}

void CsmagStateBuffer::print_info() {
    printf("\n");
    printf("info of CsmagStateBuffer object at %p:\n", this);
    printf("contained objects: %d of max %d\n", object_counter, CSMAG_BUFFER_SIZE);
    printf("first_index: %d, next_index: %d\n", first_index, next_index);
    printf("is_free_mask: 0x%x\n", is_free_mask);
    printf("is_free_mask: 0b"); print_bits(is_free_mask); printf("\n");
    printf("\n");
}

CsmagStateBuffer *CsmagStateBuffer::_singleton;

#endif

#if IS_USE_RINGBUFFER_SINGLETON_CLASSES

// since there seems to be some problem with RingBuffer using templates, let's try fixed data type int32_t for induction values:

RingBufferInt32::RingBufferInt32(int _buffer_size) {
    buffer_size = _buffer_size;
// template<typename int32_t>
// RingBufferInt32::RingBuffer(const int &_buffer_size) :
//     buffer_size(_buffer_size) {

    buf = new int32_t[buffer_size];                          // should work in C++, even with dynamic variable
    
    int i;
    for (i = 0; i < buffer_size; i++) {
        buf[i] = 0;                                     // TODO: mark as invalid???
    }
    object_counter = 0;
    first_index = INVALID_INDEX;
    next_index = 0;
    // set all available buffer slots as free
#if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
    is_free_mask = (1 << buffer_size) - 1;    // get buffer_size bits, rest is 0 (not free)
#endif

    // ..

    _singleton = this;
}

// push object to the end of ring queue buffer
// return true if pushing went successfully
bool RingBufferInt32::enqueue(int32_t new_obj) {
    //bool ret = false;
    if (is_full()) {
        // TODO: overwrite first object
        printf("WARNING! RingBufferInt32 buffer overflow, old objects get overwritten\n");
        //throw "CsmagStateBuffer buffer overflow";   // throw disabled
        //return false;
    }
    // TODO: perhaps check if it is actually free (in mask)
    buf[next_index] = new_obj;
    if (object_counter == 0) {
        first_index = next_index;   // the new first index, if no object has been stored before
    }
#if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
    mark_occupied(next_index);
#endif
    //next_index++;
    next_index = (next_index + 1) % buffer_size;
    object_counter++;
    //
    return true;
}

// pop first object at (relative_index 0, counting from first_index with 0)
int32_t RingBufferInt32::dequeue() {
    int relative_index = 0;                     
    int absolute_index = (first_index + relative_index) % buffer_size;
    // check is there is actually an object
#if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
    if (is_free(absolute_index)) {
        //throw "CsmagStateBuffer does not contain an object at the given relative_index";
        printf("ERROR! RingBufferInt32 does not contain an object at the given relative_index\n");
    }
    //
    mark_free(absolute_index);
#endif
    //first_index++;
    first_index = (first_index + 1) % buffer_size;
    object_counter--;
    return buf[absolute_index];
}

// print n as binary digits on console
template<typename INTTYPE>
void RingBufferInt32::print_bits(INTTYPE n) {
    INTTYPE p;
    for (p = ( (INTTYPE) 1) << (8 * sizeof(n) - 1); p > 0; p >>= 1) {
        printf("%d", (bool) (n & p));
    }
}

void RingBufferInt32::print_info() {
    printf("\n");
    printf("info of RingBuffer object at %p:\n", this);
    printf("contained objects: %d of max %d\n", object_counter, buffer_size);
    printf("first_index: %d, next_index: %d\n", first_index, next_index);
#if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
    printf("is_free_mask: 0x%x\n", is_free_mask);
    printf("is_free_mask: 0b"); print_bits(is_free_mask); printf("\n");
#endif
    printf("\n");
}

RingBufferInt32 *RingBufferInt32::_singleton;



// CONTINUE HERE
//InductionValueBuffer *InductionValueBuffer




// since there seems to be some problem with RingBuffer using templates, let's try fixed data type int64_t for induction values:

RingBufferUInt64::RingBufferUInt64(int _buffer_size) {
    buffer_size = _buffer_size;

    buf = new uint64_t[buffer_size];                          // should work in C++, even with dynamic variable
    
    int i;
    for (i = 0; i < buffer_size; i++) {
        buf[i] = 0;                                     // TODO: mark as invalid???
    }
    object_counter = 0;
    first_index = INVALID_INDEX;
    next_index = 0;
    // set all available buffer slots as free
#if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
    is_free_mask = (1 << buffer_size) - 1;    // get buffer_size bits, rest is 0 (not free)
#endif

    // ..

    _singleton = this;
}

// push object to the end of ring queue buffer
// return true if pushing went successfully
bool RingBufferUInt64::enqueue(uint64_t new_obj) {
    //bool ret = false;
    if (is_full()) {
        // TODO: overwrite first object
        printf("ERROR! RingBufferUInt64 buffer overflow\n");
        //throw "CsmagStateBuffer buffer overflow";   // throw disabled
        return false;
    }
    // TODO: perhaps check if it is actually free (in mask)
    buf[next_index] = new_obj;
    if (object_counter == 0) {
        first_index = next_index;   // the new first index, if no object has been stored before
    }
#if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
    mark_occupied(next_index);
#endif
    //next_index++;
    next_index = (next_index + 1) % buffer_size;
    object_counter++;
    //
    return true;
}

// pop first object at (relative_index 0, counting from first_index with 0)
uint64_t RingBufferUInt64::dequeue() {
    int relative_index = 0;                     
    int absolute_index = (first_index + relative_index) % buffer_size;
    // check is there is actually an object
#if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
    if (is_free(absolute_index)) {
        //throw "CsmagStateBuffer does not contain an object at the given relative_index";
        printf("ERROR! RingBufferUInt64 does not contain an object at the given relative_index\n");
    }
    //
    mark_free(absolute_index);
#endif
    //first_index++;
    first_index = (first_index + 1) % buffer_size;
    object_counter--;
    return buf[absolute_index];
}

// print n as binary digits on console
template<typename INTTYPE>
void RingBufferUInt64::print_bits(INTTYPE n) {
    INTTYPE p;
    for (p = ( (INTTYPE) 1) << (8 * sizeof(n) - 1); p > 0; p >>= 1) {
        printf("%d", (bool) (n & p));
    }
}

void RingBufferUInt64::print_info() {
    printf("\n");
    printf("info of RingBuffer object at %p:\n", this);
    printf("contained objects: %d of max %d\n", object_counter, buffer_size);
    printf("first_index: %d, next_index: %d\n", first_index, next_index);
#if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
    printf("is_free_mask: 0x%x\n", is_free_mask);
    printf("is_free_mask: 0b"); print_bits(is_free_mask); printf("\n");
#endif
    printf("\n");
}

RingBufferUInt64 *RingBufferUInt64::_singleton;

#endif 