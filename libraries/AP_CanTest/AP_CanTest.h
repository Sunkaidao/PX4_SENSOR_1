/*
 *
 *      Author: Eugene Shamaev
 */
#pragma once


#include <uavcan/uavcan.hpp>

#include <AP_HAL/CAN.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL_PX4/CAN.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Param/AP_Param.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

#include <AP_GPS/GPS_Backend.h>
#include <AP_Baro/AP_Baro_Backend.h>
#include <AP_Compass/AP_Compass.h>

#include <uavcan/helpers/heap_based_pool_allocator.hpp>

using namespace PX4;

#ifndef UAVCAN_NODE_POOL_SIZE
#define UAVCAN_NODE_POOL_SIZE 8192
#endif

#ifndef UAVCAN_NODE_POOL_BLOCK_SIZE
#define UAVCAN_NODE_POOL_BLOCK_SIZE 256
#endif

#ifndef UAVCAN_RCO_NUMBER
#define UAVCAN_RCO_NUMBER 18
#endif

#define AP_UAVCAN_MAX_LISTENERS 4
#define AP_UAVCAN_MAX_GPS_NODES 4
#define AP_UAVCAN_MAX_MAG_NODES 4
#define AP_UAVCAN_MAX_BARO_NODES 4

#define AP_UAVCAN_SW_VERS_MAJOR 1
#define AP_UAVCAN_SW_VERS_MINOR 0

#define AP_UAVCAN_HW_VERS_MAJOR 1
#define AP_UAVCAN_HW_VERS_MINOR 0

class AP_CanTest {
public:
    AP_CanTest();
    ~AP_CanTest();

    static const struct AP_Param::GroupInfo var_info[];

private:

    struct {
        uint16_t pulse;
        uint16_t safety_pulse;
        uint16_t failsafe_pulse;
        bool active;
    } _rco_conf[UAVCAN_RCO_NUMBER];

    bool _initialized;
    uint8_t _rco_armed;
    uint8_t _rco_safety;

    AP_HAL::Semaphore *_rc_out_sem;

    class SystemClock: public uavcan::ISystemClock, uavcan::Noncopyable {
        SystemClock()
        {
        }

        uavcan::UtcDuration utc_adjustment;
        virtual void adjustUtc(uavcan::UtcDuration adjustment)
        {
            utc_adjustment = adjustment;
        }

    public:
        virtual uavcan::MonotonicTime getMonotonic() const
        {
            uavcan::uint64_t usec = 0;
            usec = AP_HAL::micros64();
            return uavcan::MonotonicTime::fromUSec(usec);
        }
        virtual uavcan::UtcTime getUtc() const
        {
            uavcan::UtcTime utc;
            uavcan::uint64_t usec = 0;
            usec = AP_HAL::micros64();
            utc.fromUSec(usec);
            utc += utc_adjustment;
            return utc;
        }

        static SystemClock& instance()
        {
            static SystemClock inst;
            return inst;
        }
    };

    uavcan::Node<0> *_node = nullptr;

    uavcan::ISystemClock& get_system_clock();
    uavcan::ICanDriver* get_can_driver();
    uavcan::Node<0>* get_node();

/*
    // This will be needed to implement if UAVCAN is used with multithreading
    // Such cases will be firmware update, etc.
    class RaiiSynchronizer {
    public:
        RaiiSynchronizer()
        {
        }

        ~RaiiSynchronizer()
        {
        }
    };

    uavcan::HeapBasedPoolAllocator<UAVCAN_NODE_POOL_BLOCK_SIZE, AP_CanTest::RaiiSynchronizer> _node_allocator;
*/

    uint8_t _uavcan_i;

    //AP_HAL::CANManager* _parent_can_mgr;

	PX4CANManager* _parent_can_mgr;
	
	uint32_t timer;
	
public:
    void do_cyclic(void);
    bool try_init(void);

    void rco_set_safety_pwm(uint32_t chmask, uint16_t pulse_len);
    void rco_set_failsafe_pwm(uint32_t chmask, uint16_t pulse_len);
    void rco_force_safety_on(void);
    void rco_force_safety_off(void);
    void rco_arm_actuators(bool arm);
    void rco_write(uint16_t pulse_len, uint8_t ch);
	
	void send_msg(uint8_t id_mode,uint8_t frame_mode,uint8_t data_length);
	void handle_msg();

	void update();
	void init();

	/*
    void set_parent_can_mgr(AP_HAL::CANManager* parent_can_mgr)
    {
        _parent_can_mgr = parent_can_mgr;
    }
    */
};

