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
	//	added by ZhangYong 20170705 for item_int
	gcs().chan(0).set_mission_item_int(false);
	gcs().chan(1).set_mission_item_int(false);
#if MAVLINK_COMM_NUM_BUFFERS > 2
	gcs().chan(2).set_mission_item_int(false);
#endif
	//	added end

#elif FXTX_AUTH == 1
	printf("License Enabled\n");	
	
	//	added by ZhangYong 20170705 for item_int
	gcs().chan(0).set_mission_item_int(true);
	gcs().chan(1).set_mission_item_int(true);
#if MAVLINK_COMM_NUM_BUFFERS > 2
	gcs().chan(2).set_mission_item_int(true);
#endif
	//	added end

#endif

    //	added by ZhangYong 20170705
    //printf("Mavlink capabilities %x\n", hal.util->get_capabilities());
    //	added end
		//	added by ZhangYong 20170712
#if PJTPASSOSD == ENABLED
	if(nullptr != (serial_manager.find_serial(AP_SerialManager::SerialProtocol_PassOSD, 0)))
	{
		passosd.init(serial_manager);
	}
	else
	{
		
	}
#endif
	
#if PROJECTGKXN == ENABLED
	if(nullptr != (serial_manager.find_serial(AP_SerialManager::SerialProtocol_FlowMeter_GKXN,0)))
	{
		flowmeter.init(serial_manager);
	}
	//height_replace = 0;
		
#endif
	
#if BCBMONITOR == ENABLED
	
	if(nullptr != (serial_manager.find_serial(AP_SerialManager::SerialProtocol_BCBMonitor,0)))
	{
		bcbmonitor.init(serial_manager);
	}
		
#endif
	
#if BCBPMBUS == ENABLED
	printf("PROJECTBCB\n");
	printf("PMBUS\n");
		
	if(nullptr !=  serial_manager.find_serial(AP_SerialManager::SerialProtocol_BCBPMBus,0))
	{
		init_bcbpmbus();
	}
#endif
	
#if PJTPASSOSD == ENABLED
	printf("PJTPASSOSD\n");
#endif
	
	
	
#if PROJECTGKXN == ENABLED
	printf("PROJECTGKXN\n");
#endif
	
	
	//	added ebd
	
	//	added by ZhangYong 20170705
	printf("Mavlink capabilities %x\n", hal.util->get_capabilities());
	//	added end

    
    /*edit_management.words = g.edition_management;
    
    printf("major_edition = 0x%x\n", edit_management.data.major_edition);
    printf("project_edition = 0x%x\n", edit_management.data.project_edition);
    printf("minor_edition = 0x%x\n", edit_management.data.minor_edition);
    printf("revision_edition = 0x%x\n", edit_management.data.revision_edition);
    
    if(1 != edit_management.data.major_edition)
    {
    	edit_management.data.major_edition = 2;
    
    	g.edition_management.set_and_save(edit_management.words);
    }*/
	edit_management.words = g.edition_management;

	if(edit_management.data.major_edition != 2)
	{
		edit_management.data.major_edition = 2;
	}

#if PROJECTGKXN == ENABLED
	if(edit_management.data.project_edition != 1)
	{
		edit_management.data.project_edition = 1;
	}
#endif	

	//	improve the minor edition from 3 to 4
	//	failsafe rc gcs
	//	improve the minor edition from 4 to 5
	//	ABPoint, dual GPS antenna heaidng
	if(edit_management.data.minor_edition <= 4)
	{	
		edit_management.data.minor_edition = 5;
	}

	g.edition_management.set_and_save(edit_management.words);

	printf("major_edition = 0x%x\n", edit_management.data.major_edition);
	printf("project_edition = 0x%x\n", edit_management.data.project_edition);
	printf("minor_edition = 0x%x\n", edit_management.data.minor_edition);
	printf("revision_edition = 0x%x\n", edit_management.data.revision_edition);    


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
