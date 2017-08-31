#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    
    //	added by ZhangYong
    //	get board id
    #if FXTX_AUTH == 1
    	memset(auth_msg, 0, 50);
    	memset(auth_id, 0, AUTH_ID_LEN);
    
    	memset(&curr_gps_week_ms, 0, sizeof(struct current_gps_week_ms));
    //	memset(&id_para, 0, sizeof(union auth_id_para));
    
    	
    	(void)hal.util->get_system_id(auth_id);
    
    	sprintf(auth_msg, "0123456789%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n",	\
    		     (unsigned)auth_id[0], (unsigned)auth_id[1], (unsigned)auth_id[2], (unsigned)auth_id[3], (unsigned)auth_id[4], (unsigned)auth_id[5],	\
    		     (unsigned)auth_id[6], (unsigned)auth_id[7], (unsigned)auth_id[8], (unsigned)auth_id[9], (unsigned)auth_id[10], (unsigned)auth_id[11]);
    	
    
    #endif	
    //	added end 

    #if FXTX_AUTH == 0
    	printf("License Disabled\n");
    // 	//	added by ZhangYong 20170705 for item_int
    // 	gcs_chan[0]..set_mission_item_int(false);
    // 	gcs_chan[1].set_mission_item_int(false);
    // #if MAVLINK_COMM_NUM_BUFFERS > 2
    // 	gcs_chan[2].set_mission_item_int(false);
    // #endif
    // 	//	added end
    
    #elif FXTX_AUTH == 1
    
    printf("License Enabled\n");	
    // 	//	added by ZhangYong 20170705 for item_int
    // 	gcs_chan[0].set_mission_item_int(true);
    // 	gcs_chan[1].set_mission_item_int(true);
    // #if MAVLINK_COMM_NUM_BUFFERS > 2
    // 	gcs_chan[2].set_mission_item_int(true);
    // #endif
    // 	//	added end

    //	added by ZhangYong 20170705
    printf("Mavlink capabilities %x\n", hal.util->get_capabilities());
    //	added end
    
    edit_management.words = g.edition_management;
    
    printf("major_edition = 0x%x\n", edit_management.data.major_edition);
    printf("project_edition = 0x%x\n", edit_management.data.project_edition);
    printf("minor_edition = 0x%x\n", edit_management.data.minor_edition);
    printf("revision_edition = 0x%x\n", edit_management.data.revision_edition);
    
    if(1 != edit_management.data.major_edition)
    {
    	edit_management.data.major_edition = 2;
    
    	g.edition_management.set_and_save(edit_management.words);
    }


    #endif

    task.init();
    PtzControl.init();
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
    
    //baiyang added in 20170804
#if PTZ_CONTROL == ENABLED
	  PtzControl.update();
#endif
    //added end

}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
    task.update();
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
#if FXTX_AUTH== ENABLED
	 if(motors->armed())
	 {
		  local_flight_time_sec++;
	 }//	added end
#endif

}
#endif
