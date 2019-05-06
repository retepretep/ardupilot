// added by peter
// backend and implementation for CSMAG data

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>


// added by peter
// tried to use separate AP_Csmag/Csmag.h, but couldn't add this to compile process
// TEMPORARY workaround, TODO: fix this
#include <../ArduCopter/defines.h>  // for array size

// need to add backend (drivers) for CSMAG

// const AP_HAL::HAL& hal = AP_HAL::get_HAL();

template<typename T> class RingBuffer;      // forward declaration for use in Csmag

class Csmag {
public:
    Csmag();
    // we declare a virtual destructor so that CSMAG (cf. RangeFinder) drivers can
    // override with a custom destructor if need be
    virtual ~Csmag() {};

    // state for MAVLink message CSMAG to be sent
    // TODO: different name? (CsmagMessageState???)
    struct CsmagState {
        uint64_t    time_usec;
        int32_t     induction[CSMAG_INDUCTION_ARRAY_SIZE];
    };

    // update status and try to read data from MAGInterface
    int update(void);   // return number of induction values read from MAG Interface
    void synch_timestamp(uint64_t timestamp);
    
    //CsmagState *get_state() = { return csmag_state; }

    static Csmag *get_singleton(void) { return _singleton; }

    CsmagState *csmag_state;

    RingBuffer<int32_t> *induction_value_buffer;             // FIXME init with buffer size
    RingBuffer<uint64_t> *induction_value_timestamp_buffer;  //

    //uint64_t timestamp_last_synch = ~0;
    uint64_t timestamp_last_synch = 0;

    AP_HAL::UARTDriver *GetUART(void) { return uart; }

    // for debugging missing MAVLink messages

    #if ISCOUNTMESSAGES
    int count_outgoing_messages = 0;
    int count_outgoing_messages_copter = 0;
    uint64_t timestamp_first_outgoing_message = 0;
    uint64_t timestamp_last_outgoing_message = 0;
    bool is_first_count_message_sent = false;
    bool is_did_counter_reset = false;
    #endif
    #if ISTRACKMAXTIMEBETWEENMESSAGES
    uint64_t timestamp_last_message = 0;
    uint64_t timestamp_this_message = 0;
    int64_t timestamp_difference_max = -1;
    uint64_t timestamp_of_timestamp_difference_max = 0;
    #endif

private:
    static Csmag *_singleton;
    
    // return bool, if values have been read from UART successfully
    // nbytes: number of available bytes left in UART buffer,    
    // 0 ==> no data left to process,
    // n ==> still data left to process (n > 0)
    bool get_reading(uint64_t &induction_timestamp_i, int32_t &induction_value_i, int16_t &nbytes);

    // int get_reading(uint64_t &induction_timestamp_i, int32_t &induction_value_i);
    AP_HAL::UARTDriver *uart = nullptr;
    uint8_t linebuf[MAG_INTERFACE_V30_MESSAGE_SIZE + 1];    // 1 extra space to detect faulty behaviour
    uint8_t linebuf_len = 0;
};



#if IS_USE_CSMAGSTATEBUFFER

// TODO: add 2 different buffer size variables (1 config and 1 for the class)
// TODO: remove CsmagStateBuffer (since this is deprecated)
// singleton style queue buffer for Csmag::CsmagState type pointers, in order to enable reaching them from within GCS_Mavlink AND Copter class
class CsmagStateBuffer {
public:
    CsmagStateBuffer();
    virtual ~CsmagStateBuffer() {};     // cf RangeFinder backend

    bool is_full() { return is_free_mask == 0; } // no free slot? ==> isFull
    bool push(Csmag::CsmagState *new_obj);  // push object to buffer
    //Csmag::CsmagState *pop(int relative_index=0); // would be nice
    Csmag::CsmagState *pop(int relative_index);     // need to implement defragmentation
    Csmag::CsmagState *pop();                   // only pop first in queue
    int GetObjectCounter() { return object_counter; }
    uint32_t GetIsFreeMask() { return is_free_mask; }

    static const int INVALID_INDEX = -1;

    static CsmagStateBuffer *get_singleton(void) { return _singleton; }

    void print_bits(uint32_t n);
    void print_info(void);             // for debug purposes
    

private:
    void mark_free(int index) { is_free_mask |= (1 << index); }         // no checks
    void mark_occupied(int index) { is_free_mask &= ~(1 << index); }    // no checks
    bool is_free(int index) { return is_free_mask & (1 << index); }

    Csmag::CsmagState **buf;            // objects are stored here
    int first_index;           // pointing to first object
    int next_index;            // buf index of next "free" element
    uint32_t is_free_mask;              // indicating if this element DOES NOT contain a stored element
    int object_counter;
    static_assert(8*sizeof(is_free_mask) >= CSMAG_BUFFER_SIZE, "is_free_mask must have at least CSMAG_BUFFER_SIZE bits, to store the status");

    static CsmagStateBuffer *_singleton;
};

#endif


// template<typename T>
// class RingBuffer {
// public:
//     // need to use explicit?
//     // https://foonathan.net/blog/2017/10/11/explicit-assignment.html

//     explicit RingBuffer(int _buffer_size);
//     //RingBuffer(const int &_buffer_size);
//     // RingBuffer(int _buffer_size = CSMAG_INDUCTION_VALUE_BUFFER_SIZE);
//     // explicit RingBuffer(int _buffer_size = CSMAG_INDUCTION_VALUE_BUFFER_SIZE);
//     virtual ~RingBuffer() {};

// #if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
//     bool is_full() { return is_free_mask == 0; }                            // no free slot? ==> is_full
// #else
//     //bool is_full() { return first_index == next_index; }                    // would overwrite first item? ==> is_full

//     // TODO: test!
//     bool is_full() { return (first_index == next_index) && (GetObjectCounter() == GetBufferSize()); }
// #endif
//     //template<typename TT>
//     bool enqueue(T new_obj);                                               // push object to end buffer
//     //template<typename TT>
//     T dequeue();                                                           // only pop first in queue
//     int GetObjectCounter() { return object_counter; }
// #if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
//     uint32_t GetIsFreeMask() { return is_free_mask; }
// #endif
//     int GetBufferSize() { return buffer_size; }

//     static const int INVALID_INDEX = -1;

//     static RingBuffer<T> *get_singleton(void) { return _singleton; }

//     void print_bits(uint32_t n);
//     void print_info(void);             // for debug purposes
    

// private:
// #if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
//     void mark_free(int index) { is_free_mask |= (1 << index); }             // no checks
//     void mark_occupied(int index) { is_free_mask &= ~(1 << index); }        // no checks
//     bool is_free(int index) { return is_free_mask & (1 << index); }
// #endif

//     T *buf;                                                                 // objects are stored here
//     int first_index;                                                        // pointing to first object
//     int next_index;                                                         // buf index of next "free" element
// #if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
//     uint64_t is_free_mask;                                                  // indicating if this element DOES NOT contain a stored element
//     static_assert(8*sizeof(is_free_mask) >= CSMAG_BUFFER_SIZE, "is_free_mask must have at least CSMAG_BUFFER_SIZE bits, to store the status");
// #endif
//     int object_counter;
//     const int buffer_size;

//     static RingBuffer<T> *_singleton;
// };

// TODO: put into another file (no singleton class -> could be somewhere else)
template<typename T>
class RingBuffer {
public:
    // need to use explicit?
    // https://foonathan.net/blog/2017/10/11/explicit-assignment.html

    //explicit RingBuffer(int _buffer_size);
    RingBuffer(int _buffer_size);
    // RingBuffer(const int _buffer_size);
    virtual ~RingBuffer() {};

#if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
    bool is_full() { return is_free_mask == 0; }                            // no free slot? ==> is_full
#else
    //bool is_full() { return first_index == next_index; }                    // would overwrite first item? ==> is_full
    
    // TODO: test!
    bool is_full() { return (first_index == next_index) && (GetObjectCounter() == GetBufferSize()); }
#endif
    //template<typename TT>
    bool enqueue(T new_obj);                                               // push object to end buffer
    //template<typename TT>
    T dequeue();                                                           // only pop first in queue
    int GetObjectCounter() { return object_counter; }
#if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
    uint32_t GetIsFreeMask() { return is_free_mask; }
#endif
    int GetBufferSize() { return buffer_size; }

    T GetLastObject();                                                      // return object without consuming it

    static const int INVALID_INDEX = -1;

    //static RingBuffer<T> *get_singleton(void) { return _singleton; }

    void print_bits(uint32_t n);
    void print_info(void);             // for debug purposes
    int GetOverflowCounter(void) {return overflow_counter;}
    

private:
#if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
    void mark_free(int index) { is_free_mask |= (1 << index); }             // no checks
    void mark_occupied(int index) { is_free_mask &= ~(1 << index); }        // no checks
    bool is_free(int index) { return is_free_mask & (1 << index); }
#endif

    T *buf;                                                                 // objects are stored here
    int first_index;                                                        // pointing to first object
    int next_index;                                                         // buf index of next "free" element
#if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
    uint64_t is_free_mask;                                                  // indicating if this element DOES NOT contain a stored element
    static_assert(8*sizeof(is_free_mask) >= CSMAG_BUFFER_SIZE, "is_free_mask must have at least CSMAG_BUFFER_SIZE bits, to store the status");
#endif
    int object_counter;
    // const int buffer_size;
    int buffer_size;

    int overflow_counter = 0;

    //static RingBuffer<T> *_singleton;
};

// BEGIN of RingBuffer<T> definition
//  solution 2 of https://bytefreaks.net/programming-2/c/c-undefined-reference-to-templated-class-function

template<typename T>
RingBuffer<T>::RingBuffer(int _buffer_size) {
// template<typename T>
// RingBuffer<T>::RingBuffer(const int _buffer_size) {
    buffer_size = _buffer_size;
// template<typename T>
// RingBuffer<T>::RingBuffer(const int &_buffer_size) :
//     buffer_size(_buffer_size) {

    buf = new T[buffer_size];                          // should work in C++, even with dynamic variable
    
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

    //_singleton = this;
    if (ISDOVERBOSEDEBUGPRINTOUTS) {
        // hal.console->printf("called RingBuffer init\n"); // "hal is not declared in this scope"
        printf("called RingBuffer init\n");
    }
}

// push object to the end of ring queue buffer
// return true if pushing went successfully
template<typename T>
bool RingBuffer<T>::enqueue(T new_obj) {
    //bool ret = false;
    if (is_full()) {
        // TODO: overwrite first object
        printf("WARNING! RingBuffer<T> buffer overflow, old objects get overwritten, ");
        printf("buffer_size: %d\n", this->buffer_size);
        overflow_counter++;
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
template<typename T>
T RingBuffer<T>::dequeue() {
    int relative_index = 0;                     
    int absolute_index = (first_index + relative_index) % buffer_size;
    // check is there is actually an object
#if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
    if (is_free(absolute_index)) {
        //throw "CsmagStateBuffer does not contain an object at the given relative_index";
        printf("ERROR! RingBuffer does not contain an object at the given relative_index\n");
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
template<typename T>
void RingBuffer<T>::print_bits(uint32_t n) {
    uint32_t p;
    for (p = 1 << (8 * sizeof(n) - 1); p > 0; p >>= 1) {
        printf("%d", (bool) (n & p));
    }
}

template<typename T>
void RingBuffer<T>::print_info() {
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

template<typename T>
T RingBuffer<T>::GetLastObject() {
    return buf[(next_index + buffer_size - 1) % buffer_size];       // next points to first free object
}

// END of RingBuffer<T> definition

#if IS_USE_RINGBUFFER_SINGLETON_CLASSES

/*
class RingBufferInt32 : public RingBuffer<int32_t> {
    using RingBuffer<int32_t>::RingBuffer;
};
*/

// since there seems to be some problem with RingBuffer using templates, let's try fixed data type int32_t for induction values:

class RingBufferInt32 {
public:
    // need to use explicit?
    // https://foonathan.net/blog/2017/10/11/explicit-assignment.html

    RingBufferInt32(int _buffer_size);
    //RingBuffer(const int &_buffer_size);
    // RingBuffer(int _buffer_size = CSMAG_INDUCTION_VALUE_BUFFER_SIZE);
    // explicit RingBuffer(int _buffer_size = CSMAG_INDUCTION_VALUE_BUFFER_SIZE);
    virtual ~RingBufferInt32() {};

#if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
    bool is_full() { return is_free_mask == 0; }                            // no free slot? ==> is_full
#else
    // would overwrite first item? ==> either FULL or EMPTY!
    bool is_full() { return (first_index == next_index) && (GetObjectCounter() == GetBufferSize()); }
#endif
    bool enqueue(int32_t new_obj);                                               // push object to end buffer
    int32_t dequeue();                                                           // only pop first in queue
    int GetObjectCounter() { return object_counter; }
#if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
    uint32_t GetIsFreeMask() { return is_free_mask; }
#endif
    int GetBufferSize() { return buffer_size; }

    static const int INVALID_INDEX = -1;

    static RingBufferInt32 *get_singleton(void) { return _singleton; }

    template<typename INTTYPE>
    void print_bits(INTTYPE n);
    void print_info(void);             // for debug purposes
    

private:
#if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
    void mark_free(int index) { is_free_mask |= (1 << index); }             // no checks
    void mark_occupied(int index) { is_free_mask &= ~(1 << index); }        // no checks
    bool is_free(int index) { return is_free_mask & (1 << index); }
#endif

    int32_t *buf;                                                                 // objects are stored here
    int first_index;                                                        // pointing to first object
    int next_index;                                                         // buf index of next "free" element
#if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
    __uint128_t is_free_mask;                                                  // indicating if this element DOES NOT contain a stored element
    static_assert(8*sizeof(is_free_mask) >= CSMAG_INDUCTION_VALUE_BUFFER_SIZE, "is_free_mask must have at least CSMAG_BUFFER_SIZE bits, to store the status");
#endif
    int object_counter;
    /*const*/ int buffer_size;

    static RingBufferInt32 *_singleton;
};



// CONTINUE HERE
/*
class InductionValueBuffer : RingBufferInt32 {
    using RingBufferInt32::RingBufferInt32;
};
*/




class RingBufferUInt64 {
public:
    // need to use explicit?
    // https://foonathan.net/blog/2017/10/11/explicit-assignment.html

    RingBufferUInt64(int _buffer_size);
    //RingBuffer(const int &_buffer_size);
    // RingBuffer(int _buffer_size = CSMAG_INDUCTION_VALUE_BUFFER_SIZE);
    // explicit RingBuffer(int _buffer_size = CSMAG_INDUCTION_VALUE_BUFFER_SIZE);
    virtual ~RingBufferUInt64() {};

#if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
    bool is_full() { return is_free_mask == 0; }                            // no free slot? ==> is_full
#else
    // would overwrite first item? ==> either FULL or EMPTY!
    bool is_full() { return (first_index == next_index) && (GetObjectCounter() == GetBufferSize()); }
#endif
    bool enqueue(uint64_t new_obj);                                               // push object to end buffer
    uint64_t dequeue();                                                           // only pop first in queue
    int GetObjectCounter() { return object_counter; }
#if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
    __uint128_t GetIsFreeMask() { return is_free_mask; }
#endif
    int GetBufferSize() { return buffer_size; }

    static const int INVALID_INDEX = -1;

    static RingBufferUInt64 *get_singleton(void) { return _singleton; }

    template<typename INTTYPE>
    void print_bits(INTTYPE n);
    void print_info(void);             // for debug purposes
    

private:
#if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
    void mark_free(int index) { is_free_mask |= (1 << index); }             // no checks
    void mark_occupied(int index) { is_free_mask &= ~(1 << index); }        // no checks
    bool is_free(int index) { return is_free_mask & (1 << index); }
#endif

    uint64_t *buf;                                                                 // objects are stored here
    int first_index;                                                        // pointing to first object
    int next_index;                                                         // buf index of next "free" element
#if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
    __uint128_t is_free_mask;                                                  // indicating if this element DOES NOT contain a stored element
    static_assert(8*sizeof(is_free_mask) >= CSMAG_INDUCTION_VALUE_BUFFER_SIZE, "is_free_mask must have at least CSMAG_BUFFER_SIZE bits, to store the status");
#endif
    int object_counter;
    /*const*/ int buffer_size;

    static RingBufferUInt64 *_singleton;
};

#endif 
