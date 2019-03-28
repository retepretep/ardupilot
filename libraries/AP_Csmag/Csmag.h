// added by peter
// backend and implementation for CSMAG data

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>

//#include <defines>
#include "../defines.h"

// need to add backend (drivers) for CSMAG
class Csmag {
public:
    Csmag();                                    // ctor
    // we declare a virtual destructor so that CSMAG (cf. RangeFinder) drivers can
    // override with a custom destructor if need be
    virtual ~Csmag(void) {};                    // dtor
    //
    struct CsmagState {
        uint64_t    time_usec;
        int32_t     induction[CSMAG_INDUCTION_ARRAY_SIZE];
        // TODO: array with fixed size of pointer?
    };
    //
    CsmagState *get_state(void) = { return csmag_state; }

private:
     *csmag_state;
}