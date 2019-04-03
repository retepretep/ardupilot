/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>

// Maximum number of range finder instances available on this platform
#define RANGEFINDER_MAX_INSTANCES 2
#define RANGEFINDER_GROUND_CLEARANCE_CM_DEFAULT 10
#define RANGEFINDER_PREARM_ALT_MAX_CM           200
#define RANGEFINDER_PREARM_REQUIRED_CHANGE_CM   50

class AP_RangeFinder_Backend;

class RangeFinder
{
    friend class AP_RangeFinder_Backend;

public:
    RangeFinder(AP_SerialManager &_serial_manager, enum Rotation orientation_default);

    /* Do not allow copies */
    RangeFinder(const RangeFinder &other) = delete;
    RangeFinder &operator=(const RangeFinder&) = delete;

    // RangeFinder driver types
    enum RangeFinder_Type {
        RangeFinder_TYPE_NONE   = 0,
        RangeFinder_TYPE_ANALOG = 1,
        RangeFinder_TYPE_MBI2C  = 2,
        RangeFinder_TYPE_PLI2C  = 3,
        RangeFinder_TYPE_PX4    = 4,
        RangeFinder_TYPE_PX4_PWM= 5,
        RangeFinder_TYPE_BBB_PRU= 6,
        RangeFinder_TYPE_LWI2C  = 7,
        RangeFinder_TYPE_LWSER  = 8,
        RangeFinder_TYPE_BEBOP  = 9,
        RangeFinder_TYPE_MAVLink = 10,
        RangeFinder_TYPE_ULANDING= 11,
        RangeFinder_TYPE_LEDDARONE = 12,
        RangeFinder_TYPE_MBSER  = 13,
        RangeFinder_TYPE_TRI2C  = 14,
        RangeFinder_TYPE_PLI2CV3= 15,
        RangeFinder_TYPE_VL53L0X = 16,
        RangeFinder_TYPE_NMEA = 17,
        RangeFinder_TYPE_WASP = 18,
        RangeFinder_TYPE_BenewakeTF02 = 19,
        RangeFinder_TYPE_BenewakeTFmini = 20,
        RangeFinder_TYPE_PLI2CV3HP = 21,
    };

    enum RangeFinder_Function {
        FUNCTION_LINEAR    = 0,
        FUNCTION_INVERTED  = 1,
        FUNCTION_HYPERBOLA = 2
    };

    enum RangeFinder_Status {
        RangeFinder_NotConnected = 0,
        RangeFinder_NoData,
        RangeFinder_OutOfRangeLow,
        RangeFinder_OutOfRangeHigh,
        RangeFinder_Good
    };

    // The RangeFinder_State structure is filled in by the backend driver
    struct RangeFinder_State {
        uint16_t               distance_cm; // distance: in cm
        uint16_t               voltage_mv;  // voltage in millivolts,
                                            // if applicable, otherwise 0
        enum RangeFinder_Status status;     // sensor status
        uint8_t                range_valid_count;   // number of consecutive valid readings (maxes out at 10)
        bool                   pre_arm_check;   // true if sensor has passed pre-arm checks
        uint16_t               pre_arm_distance_min;    // min distance captured during pre-arm checks
        uint16_t               pre_arm_distance_max;    // max distance captured during pre-arm checks

        AP_Int8  type;
        AP_Int8  pin;
        AP_Int8  ratiometric;
        AP_Int8  stop_pin;
        AP_Int16 settle_time_ms;
        AP_Float scaling;
        AP_Float offset;
        AP_Int8  function;
        AP_Int16 min_distance_cm;
        AP_Int16 max_distance_cm;
        AP_Int8  ground_clearance_cm;
        AP_Int8  address;
        AP_Vector3f pos_offset; // position offset in body frame
        AP_Int8  orientation;
        const struct AP_Param::GroupInfo *var_info;
    };

    static const struct AP_Param::GroupInfo *backend_var_info[RANGEFINDER_MAX_INSTANCES];
    
    AP_Int16 _powersave_range;

    // parameters for each instance
    static const struct AP_Param::GroupInfo var_info[];
    
    // Return the number of range finder instances
    uint8_t num_sensors(void) const {
        return num_instances;
    }

    // detect and initialise any available rangefinders
    void init(void);

    // update state of all rangefinders. Should be called at around
    // 10Hz from main loop
    void update(void);

    // Handle an incoming DISTANCE_SENSOR message (from a MAVLink enabled range finder)
    void handle_msg(mavlink_message_t *msg);

    // return true if we have a range finder with the specified orientation
    bool has_orientation(enum Rotation orientation) const;

    // find first range finder instance with the specified orientation
    AP_RangeFinder_Backend *find_instance(enum Rotation orientation) const;

    AP_RangeFinder_Backend *get_backend(uint8_t id) const;

    // methods to return a distance on a particular orientation from
    // any sensor which can current supply it
    uint16_t distance_cm_orient(enum Rotation orientation) const;
    uint16_t voltage_mv_orient(enum Rotation orientation) const;
    int16_t max_distance_cm_orient(enum Rotation orientation) const;
    int16_t min_distance_cm_orient(enum Rotation orientation) const;
    int16_t ground_clearance_cm_orient(enum Rotation orientation) const;
    MAV_DISTANCE_SENSOR get_mav_distance_sensor_type_orient(enum Rotation orientation) const;
    RangeFinder_Status status_orient(enum Rotation orientation) const;
    bool has_data_orient(enum Rotation orientation) const;
    uint8_t range_valid_count_orient(enum Rotation orientation) const;
    const Vector3f &get_pos_offset_orient(enum Rotation orientation) const;

    /*
      set an externally estimated terrain height. Used to enable power
      saving (where available) at high altitudes.
     */
    void set_estimated_terrain_height(float height) {
        estimated_terrain_height = height;
    }

    /*
      returns true if pre-arm checks have passed for all range finders
      these checks involve the user lifting or rotating the vehicle so that sensor readings between
      the min and 2m can be captured
     */
    bool pre_arm_check() const;

    static RangeFinder *get_singleton(void) { return _singleton; }


private:
    static RangeFinder *_singleton;

    RangeFinder_State state[RANGEFINDER_MAX_INSTANCES];
    AP_RangeFinder_Backend *drivers[RANGEFINDER_MAX_INSTANCES];
    uint8_t num_instances:3;
    float estimated_terrain_height;
    AP_SerialManager &serial_manager;
    Vector3f pos_offset_zero;   // allows returning position offsets of zero for invalid requests

    void detect_instance(uint8_t instance, uint8_t& serial_instance);
    void update_instance(uint8_t instance);  

    bool _add_backend(AP_RangeFinder_Backend *driver);
};

// added by peter
// tried to use separate AP_Csmag/Csmag.h, but couldn't add this to compile process
// TEMPORARY workaround, TODO: fix this
#include <../ArduCopter/defines.h>  // for array size

// need to add backend (drivers) for CSMAG



class Csmag {
public:
    Csmag();
    // we declare a virtual destructor so that CSMAG (cf. RangeFinder) drivers can
    // override with a custom destructor if need be
    virtual ~Csmag() {};
    //
    struct CsmagState {
        uint64_t    time_usec;
        int32_t     induction[CSMAG_INDUCTION_ARRAY_SIZE];
    };
    
    //CsmagState *get_state() = { return csmag_state; }

    static Csmag *get_singleton(void) { return _singleton; }

    CsmagState *csmag_state;

private:
    static Csmag *_singleton;
};





// TODO: add 2 different buffer size variables (1 config and 1 for the class)

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

    static const int INVALID_INDEX = -1;

    //static RingBuffer<T> *get_singleton(void) { return _singleton; }

    void print_bits(uint32_t n);
    void print_info(void);             // for debug purposes
    

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
    if (ISDOTEMPVERBOSEDEBUG) {
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
        printf("WARNING! RingBuffer<T> buffer overflow, old objects get overwritten\n");
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

// END of RingBuffer<T> definition


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
