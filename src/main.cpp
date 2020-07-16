/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : Run Elmo driver 
 * ifname is NIC interface, f.e. rteth0(RTnet socket)
 * cycletime in us, f.e. 1000(1ms)
 *
 * (c)Arthur Kim Hyeon Seok 2020
 */

// **********Basic libraries*********//
#include <sys/mman.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <iostream>
#include <stdio.h>
#include <sys/time.h>
#include <pthread.h>
#include <inttypes.h>

// **********ROS libraries*********//
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>

// **********SOEM library*********//
#include "ethercat.h"
#include "pdo_def.h"
#include "ecat_dc.h"
#include "slave_info.h"

// **********Xenomai libraries*********//
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <alchemy/mutex.h>

#include <alchemy/sem.h>
#include <boilerplate/trace.h>
#include <xenomai/init.h>

#define NSEC_PER_SEC 1000000000
#define EC_TIMEOUTMON 500

#define NUM_OF_ELMO 3
//#define _USE_DC

//*************** 1. Variables ****************/
// Ethercat
char ecat_ifname[32] = "rteth0";
ELMO_Drive_pt ELMO_drive_pt[NUM_OF_ELMO];
bool needlf;
bool inOP;
uint32_t ob;
uint16_t ob2;
uint8_t ob3;
char IOmap[4096];
int oloop, iloop, wkc_count;
int expectedWKC;
volatile int wkc;
unsigned int cycle_ns = 1000000; // Control Cycle 1[ms]
int recv_fail_cnt = 0;

// loop flag (main & thread)
int main_run = 1;
int motion_run = 1;
int print_run = 1;
int ros_run = 1;

// Servo
int ServoState = 0;
int servo_ready = 0;
int sys_ready = 0;
int started[NUM_OF_ELMO] = {0,0,0};
uint16_t controlword = 0;
long stick = 0;
unsigned long ready_cnt = 0;

//// Elmo setting
UINT16 maxTorque = 3500;

//Save
#define SAVE_LENGTH 8    //The number of data
#define SAVE_COUNT 3600 //Save Time = 3600000[ms]=3600[s]
unsigned int save_cnt = 0;
double save_array[SAVE_COUNT][SAVE_LENGTH];

// ROS Communication Param
ros::Subscriber S_Mode;
ros::Subscriber S_Stop;
ros::Publisher P_data;
std_msgs::Float64MultiArray m_data;
int ControlMode=0;

//Xenomai time variablse
RT_TASK RT_task1;
RT_TASK RT_task2;
RT_TASK RT_task3;
RTIME now1, previous1; // Ethercat time
RTIME now2, previous2; // Thread 1 cycle time
RTIME now3, previous3; // Thread 2 cycle time
double del_time1 = 0.0; // Ethercat time
double del_time2 = 0.0; // Thread 1 cycle time
double del_time3 = 0.0; // Thread 2 cycle time
double max_time = 0.0;

// Joint_info
typedef struct Joint {
    //* Current information
    double currentAngle=0.0;
    double currentVel=0.0;
    double currentAcc=0.0;
    double torque=0.0;

    //* Reference information
    double refAngle=0.0;
    double refVel=0.0;
    double refAcc=0.0;

    //* Control P I D gain
    double gain_P=0.0;
    double gain_I=0.0;
    double gain_D=0.0;
} JOINT;

JOINT* joint;

double actual_joint_pos[NUM_OF_ELMO] = {0.0,0.0,0.0};
double ABS_actual_joint_pos[NUM_OF_ELMO] = {0.0,0.0,0.0};
double Incre_actual_joint_pos[NUM_OF_ELMO] = {0.0,0.0,0.0};
double Incre_actual_joint_pos_offset[NUM_OF_ELMO] = {0.0,0.0,0.0};
double actual_joint_vel[NUM_OF_ELMO] = {0.0,0.0,0.0};
    
bool Encoder_Reset_Flag = true;
int Gear[NUM_OF_ELMO] = {50, 50, 50};
int Ratio[NUM_OF_ELMO] = {1, 1, 1};
double ratedCur[NUM_OF_ELMO] = {2.85, 2.85, 8.9};
double Kt[NUM_OF_ELMO] = {0.159, 0.159, 0.156};
int32_t Resolution[NUM_OF_ELMO] = {65536, 65536, 16384}; //16384(2^14)
    
//**********************************************//
//*************** 2. Functions ****************//
void ServoOn(void); // : Servo ON
void ServoOff(void); // : Servo Off
void motion_task(void* arg); // : Thread 1
void print_task(void* arg); // : Thread 2
void catch_signal(int sig); // : Catch "Ctrl + C signal"
bool ecat_init(void);

double Count2Deg(int Gear_Ratio, INT32 Count);
double Count2DegDot(int Gear_Ratio, INT32 CountPerSec);
double Count2Rad(int Gear_Ratio, INT32 Count);
double Count2RadDot(int Gear_Ratio, INT32 CountPerSec);
double Count2Rad_ABS(int _Resolution, INT32 Count);
INT32 Count_tf(int _Ratio, INT32 _Count_in);
INT16 Tor2Cur(double OutputTorque, double _Kt, double _ratedCur);

void Load(void); // : parameter initial setting
void EncoderRead(void); // : q, q_dot Read
void jointController(void);
void DataSave(void);
void FileSave(void);
void Max_Time_Save(double now_time);

// ROS function
void Callback1(const std_msgs::Int32 &msg);

void ROSMsgPublish(void);
static int RS3_write8(uint16 slave, uint16 index, uint8 subindex, uint8 value);
static int RS3_write16(uint16 slave, uint16 index, uint8 subindex, uint16 value);
static int RS3_write32(uint16 slave, uint16 index, uint8 subindex, uint32 value);

int main(int argc, char* argv[]) {

    printf(" ROS Setting ...\n");
    ros::init(argc, argv, "elmo_pkgs");
    ros::NodeHandle nh;
    
    signal(SIGINT, catch_signal);
    signal(SIGTERM, catch_signal); 
    mlockall(MCL_CURRENT | MCL_FUTURE);
 
    S_Mode = nh.subscribe("Mode", 1, Callback1);
    P_data = nh.advertise<std_msgs::Float64MultiArray>("ROS_DATA", 1);
    m_data.data.resize(10);
    sleep(1);

    printf(" Init main ...\n");
    sleep(1);

    // Display Adapter name
    printf(" Use default adapter %s ...\n", ecat_ifname);
    sleep(1);

    // Initial Setting
    printf(" Load Params...\n");
    Load();

    // Thread Setting Start
    printf("Create Thread ...\n");
    for (int i = 1; i < 4; ++i) {
        printf("%d...\n", i);
        sleep(1);
    }

    rt_task_create(&RT_task1, "Motion_task", 0, 99, 0);
    rt_task_create(&RT_task2, "Print_task", 0, 80, 0);

    rt_task_start(&RT_task1, &motion_task, NULL);
    rt_task_start(&RT_task2, &print_task, NULL);
    // Thread Setting End
    
    ros::Rate loop_rate(1000);
    while(ros::ok()){
        ROSMsgPublish();
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}

void motion_task(void* arg) {
    //printf("Motion Thread Start \n");

    if (ecat_init() == false) {
        motion_run = 0;
    }
    rt_task_sleep(1e6);

#ifdef _USE_DC
    //for dc computation
    long long toff;
    long long cur_DCtime = 0, max_DCtime = 0;
    unsigned long long cur_dc32 = 0, pre_dc32 = 0;
    int32_t shift_time = 380000; //dc event shifted compared to master reference clock
    long long diff_dc32;

    for (int i = 0; i < NUM_OF_ELMO; ++i) {
        ec_dcsync0(1 + i, TRUE, cycle_ns, 0); // SYNC0,1 on slave 1
    }

    RTIME cycletime = cycle_ns, cur_time = 0;
    RTIME cur_cycle_cnt = 0, cycle_time;
    RTIME remain_time, dc_remain_time;
    toff = 0;
    RTIME rt_ts;

    //get DC time for first time
    ec_send_processdata();

    cur_time = rt_timer_read(); //get current master time
    cur_cycle_cnt = cur_time / cycle_ns; //calcualte number of cycles has passed
    cycle_time = cur_cycle_cnt * cycle_ns;
    remain_time = cur_time % cycle_ns; //remain time to next cycle, test only

    rt_printf("cycle_cnt=%lld\n", cur_cycle_cnt);
    rt_printf("remain_time=%lld\n", remain_time);

    wkc = ec_receive_processdata(EC_TIMEOUTRET); //get reference DC time

    cur_dc32 = (uint32_t) (ec_DCtime & 0xffffffff); //only consider first 32-bit
    dc_remain_time = cur_dc32 % cycletime; //remain time to next cycle of REF clock, update to master

    //remain time to next cycle of REF clock, update to master
    rt_ts = cycle_time + dc_remain_time; //update master time to REF clock
    rt_printf("dc remain_time=%lld\n", dc_remain_time);
    rt_task_sleep_until(rt_ts);

#else  //nonDC
    ec_send_processdata();
    rt_task_set_periodic(NULL, TM_NOW, cycle_ns);

#endif
    while (motion_run) {
        //       rt_mutex_acquire(&mutex_desc, TM_INFINITE);
#ifdef _USE_DC     
        rt_ts += (RTIME) (cycle_ns + toff);
        rt_task_sleep_until(rt_ts);
#else  
        rt_task_wait_period(NULL);
#endif
        previous2 = now2;
        previous1 = rt_timer_read();

        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
        if (wkc < 3 * (NUM_OF_ELMO)) {
            recv_fail_cnt++;
        }

#ifdef _USE_DC    
        cur_dc32 = (uint32_t) (ec_DCtime & 0xffffffff); //use 32-bit only
        if (cur_dc32 > pre_dc32) { //normal case
            diff_dc32 = cur_dc32 - pre_dc32;
        } else { //32-bit data overflow
            diff_dc32 = (0xffffffff - pre_dc32) + cur_dc32;
        }
        pre_dc32 = cur_dc32;
        cur_DCtime += diff_dc32;
        toff = dc_pi_sync(cur_DCtime, cycletime, shift_time);
        if (cur_DCtime > max_DCtime) {
            max_DCtime = cur_DCtime;
        }
#endif                   
        ready_cnt++;
        if (ready_cnt >= 1000) {
            ready_cnt = 6000;
            sys_ready = 1;
        }
        if (sys_ready == 0) {
            ServoOn();
            if (ServoState == (1 << NUM_OF_ELMO) - 1) //all servos are in ON state
            {
                if (servo_ready == 0) {
                    servo_ready = 1;
                }
            }
            if (servo_ready) {
                ready_cnt++;
            }
            if (ready_cnt >= 5000) {
                sys_ready = 1;
                
            }
        } else { // realtime action...
            EncoderRead();
            jointController();
        }

        now2 = rt_timer_read();
        now1 = rt_timer_read();

        del_time1 = (double) (now1 - previous1) / 1000000;
        del_time2 = (double) (now2 - previous2) / 1000000;
        Max_Time_Save(del_time1);
        DataSave();
    }
    //    rt_task_sleep(cycle_ns);
#ifdef _USE_DC
    for (int i = 0; i < NUM_OF_ELMO; ++i)
        ec_dcsync0(i + 1, FALSE, 0, 0); // SYNC0,1 on slave 1
#endif 
    for (int i = 0; i < NUM_OF_ELMO; ++i) {
        ELMO_drive_pt[i].ptOutParam->ControlWord = 0; //Servo OFF (Disable voltage, transition#9)
    }
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);

    rt_task_sleep(cycle_ns);

    //rt_printf("End simple test, close socket\n");
    /* stop SOEM, close socket */
    printf("Request safe operational state for all slaves\n");
    ec_slave[0].state = EC_STATE_SAFE_OP;
    /* request SAFE_OP state for all slaves */
    ec_writestate(0);
    /* wait for all slaves to reach state */
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

    ec_slave[0].state = EC_STATE_PRE_OP;
    /* request SAFE_OP state for all slaves */
    ec_writestate(0);
    /* wait for all slaves to reach state */
    ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

    ec_close();
}

void print_task(void* arg) {

    rt_task_set_periodic(NULL, TM_NOW, cycle_ns * 100);

    //rt_printf("Print Thread Start \n");

    while (print_run) {
        previous3 = now3;
        //rt_mutex_acquire(&mutex_desc, TM_INFINITE);
        rt_task_wait_period(NULL);

        if (inOP == TRUE) {
            if (!sys_ready) {
                if (stick == 0)
                    //rt_printf("waiting for system ready...\n");
                    if (stick % 10 == 0)
                        // rt_printf("%i\n", stick / 10);
                        stick++;
            } else {
                rt_printf("_______________________________\n");
                rt_printf("Thread_time : %f [ms] / %f [ms] / %f [ms] \n", del_time1, del_time2, del_time3);
                rt_printf("max_time : %f [ms]\n", max_time);
                rt_printf("_________________________________________\n");
                rt_printf("Status word = 0x%X / 0x%X / 0x%X \n", ELMO_drive_pt[0].ptInParam->StatusWord, ELMO_drive_pt[1].ptInParam->StatusWord, ELMO_drive_pt[2].ptInParam->StatusWord);
                rt_printf("actual_q(degree) = %3f / %3f / %3f \n", actual_joint_pos[0] * R2D, actual_joint_pos[1] * R2D, actual_joint_pos[2] * R2D);
                rt_printf("actual_q_dot=%3f / %3f / %3f\n", actual_joint_vel[0], actual_joint_vel[1], actual_joint_vel[2]);
                rt_printf("_________________________________________\n");
            }
        }
        // rt_mutex_release(&mutex_desc);
        now3 = rt_timer_read();
        del_time3 = (double) (now3 - previous3) / 1000000;
    }
}

void catch_signal(int sig) {
    
    printf("Program END...\n");
    
    FileSave();
    
    rt_task_delete(&RT_task1);
    rt_task_delete(&RT_task2);
        
    ros::shutdown();
}

void ServoOn(void) {
    //servo-on	
    for (int i = 0; i < NUM_OF_ELMO; i++) {
        controlword = 0;
        started[i] = ServoOn_GetCtrlWrd(ELMO_drive_pt[i].ptInParam->StatusWord, &controlword);
        ELMO_drive_pt[i].ptOutParam->ControlWord = controlword;
        if (started[i]) ServoState |= (1 << i);
    }
}

void ServoOff(void) {
    //printf("function is Servo_off");
    //Servo OFF
    for (int i = 0; i < NUM_OF_ELMO; i++) {
        ELMO_drive_pt[i].ptOutParam->ControlWord = 0; //Servo OFF (Disable voltage, transition#9)
    }
}

void EncoderRead(void) {
    if (Encoder_Reset_Flag == true) {
        for (int i = 0; i < NUM_OF_ELMO; i++) {
            ABS_actual_joint_pos[i] = Count2Rad_ABS(Resolution[i], Count_tf(Ratio[i], ELMO_drive_pt[i].ptInParam->AuxiliaryPositionActualValue));
            Incre_actual_joint_pos_offset[i] = Count2Rad(Gear[i], ELMO_drive_pt[i].ptInParam->PositionActualValue);
            actual_joint_pos[i] = ABS_actual_joint_pos[i] + Incre_actual_joint_pos[i];
        }
        Encoder_Reset_Flag = false;
    }

    for (int i = 0; i < NUM_OF_ELMO; i++) {
        Incre_actual_joint_pos[i] = Count2Rad(Gear[i], ELMO_drive_pt[i].ptInParam->PositionActualValue) - Incre_actual_joint_pos_offset[i];
        actual_joint_pos[i] = ABS_actual_joint_pos[i] + Incre_actual_joint_pos[i];
        actual_joint_vel[i] = Count2RadDot(Gear[i], ELMO_drive_pt[i].ptInParam -> VelocityActualValue);
        
        joint[i].currentAngle=actual_joint_pos[i];
        joint[i].currentVel=actual_joint_vel[i];
    }
}


void Load(void) {
    joint = new JOINT[NUM_OF_ELMO]; 
    for(int i=0; i<NUM_OF_ELMO; i++){
     joint[i].torque = 0;    
     joint[i].currentAngle = 0;
     joint[i].currentVel = 0;
    }
}

void jointController(void) {

    for (int i = 0; i < NUM_OF_ELMO; i++) {
        if (joint[i].torque >= 3000) {
            joint[i].torque = 3000;
        } else if (joint[i].torque <= -3000) {
            joint[i].torque = -3000;
        }
    }

    for (int i = 0; i < NUM_OF_ELMO; i++) {
        ELMO_drive_pt[i].ptOutParam->TargetTorque = Tor2Cur(joint[i].torque, Kt[i], ratedCur[i]);
    }
}

void Callback1(const std_msgs::Int32 &msg) {
    ControlMode=msg.data;
}


void ROSMsgPublish(void) {
    
    m_data.data[0] = del_time1;
    m_data.data[1] = actual_joint_pos[0] * R2D;
    m_data.data[2] = actual_joint_pos[1] * R2D;
    m_data.data[3] = actual_joint_pos[2] * R2D;

    P_data.publish(m_data);    
}

void DataSave(void) {

    save_array[save_cnt][0] = del_time1;
    save_array[save_cnt][1] = actual_joint_pos[0] * R2D;
    save_array[save_cnt][2] = actual_joint_pos[1] * R2D;
    save_array[save_cnt][3] = actual_joint_pos[2] * R2D;
   
    if (save_cnt < SAVE_COUNT - 1){
        save_cnt++;
    }
}

void FileSave(void) {
    FILE *fp;

    fp = fopen("Data.txt", "w");
    
    for (int j = 0; j <= SAVE_COUNT - 1; ++j) {
        for (int i = 0; i <= SAVE_LENGTH - 1; ++i) {
            fprintf(fp, "%f\t", save_array[j][i]);
        }
        fprintf(fp, "%f\n", save_array[j][SAVE_LENGTH - 1]);
    }

    fclose(fp);
}

bool ecat_init(void) {

    needlf = FALSE;
    inOP = FALSE;

    printf("Starting simple test\n");

    if (ec_init(ecat_ifname)) {
        
        printf("ec_init on %s succeeded.\n", ecat_ifname);

        if (ec_config_init(FALSE) > 0) {
            printf("%d slaves found and configured.\n", ec_slavecount);
            for (int i = 0; i < NUM_OF_ELMO; ++i) {
                printf("Has CA? %d %s\n", i + 1, ec_slave[i + 1].CoEdetails & ECT_COEDET_SDOCA ? "true" : "false");
            }

            for (int i = 1; i < NUM_OF_ELMO; ++i) {
                ec_slave[i + 1].CoEdetails ^= ECT_COEDET_SDOCA;
            }

            ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

            // PDO re-mapping //
            for (int k = 0; k < NUM_OF_ELMO; ++k) {
                if (ec_slavecount >= 1) {
                    wkc += RS3_write8(k + 1, 0x1c12, 0x0000, 0x00); //(slave, index, subindex, value)
                    wkc += RS3_write8(k + 1, 0x1608, 0x0000, 0x00);

                    wkc += RS3_write32(k + 1, 0x1608, 0x0001, 0x607A0020);
                    wkc += RS3_write32(k + 1, 0x1608, 0x0002, 0x60FF0020);
                    wkc += RS3_write32(k + 1, 0x1608, 0x0003, 0x60710010);
                    wkc += RS3_write32(k + 1, 0x1608, 0x0004, 0x60720010);
                    wkc += RS3_write32(k + 1, 0x1608, 0x0005, 0x60400010);
                    wkc += RS3_write32(k + 1, 0x1608, 0x0006, 0x60600008);
                    wkc += RS3_write8(k + 1, 0x1608, 0x0000, 0x06);


                    wkc += RS3_write16(k + 1, 0x1c12, 0x0001, 0x1608); //  (row,PDO) 
                    wkc += RS3_write8(k + 1, 0x1c12, 0x0000, 0x01); //  (index,Row)

                    ////////////////////////////////////////////////////////////////////////////

                    wkc += RS3_write8(k + 1, 0x1c13, 0x0000, 0x00); //(slave, index, subindex, value)
                    wkc += RS3_write8(k + 1, 0x1a07, 0x0000, 0x00);

                    wkc += RS3_write32(k + 1, 0x1a07, 0x0001, 0x60640020);
                    wkc += RS3_write32(k + 1, 0x1a07, 0x0002, 0x60FD0020);
                    wkc += RS3_write32(k + 1, 0x1a07, 0x0003, 0x606C0020);
                    wkc += RS3_write32(k + 1, 0x1a07, 0x0004, 0x60410010);
                    wkc += RS3_write32(k + 1, 0x1a07, 0x0005, 0x20a00020);

                    wkc += RS3_write8(k + 1, 0x1a07, 0x0000, 0x05);
                    wkc += RS3_write16(k + 1, 0x1c13, 0x0001, 0x1a07); //  (row,PDO) 
                    wkc += RS3_write8(k + 1, 0x1c13, 0x0000, 0x01); //  (index,Row)
                }
            }

            ec_config_map(&IOmap);

#ifdef _USE_DC
            ec_configdc();
#endif  
            printf("Slaves mapped, state to SAFE_OP.\n");
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

#ifdef _USE_DC
            printf("DC capable : %d\n", ec_configdc());
#endif

            slaveinfo(ecat_ifname);
            
            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
            printf("segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);
            printf("Request operational state for all slaves\n");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);
            ec_slave[0].state = EC_STATE_OPERATIONAL;

            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);

            ec_writestate(0);
            ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE); //wait for OP

            if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
                printf("Operational state reached for all slaves.\n");
                wkc_count = 0;

                for (int k = 0; k < NUM_OF_ELMO; ++k) {
                    ELMO_drive_pt[k].ptOutParam = (ELMO_DRIVE_RxPDO_t*) ec_slave[k + 1].outputs;
                    ELMO_drive_pt[k].ptInParam = (ELMO_DRIVE_TxPDO_t*) ec_slave[k + 1].inputs;
                    ELMO_drive_pt[k].ptOutParam->ModeOfOperation = OP_MODE_CYCLIC_SYNC_TORQUE;
                    ELMO_drive_pt[k].ptOutParam->MaxTorque = maxTorque;
                }
                inOP = TRUE;
            } else {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for (int i = 1; i <= ec_slavecount; ++i) {
                    if (ec_slave[i].state != EC_STATE_OPERATIONAL) {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                                i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
                for (int i = 0; i < NUM_OF_ELMO; ++i)
                    ec_dcsync01(i + 1, FALSE, 0, 0, 0); // SYNC0,1 
            }

        } else {
            printf("No slaves found!\n");
            inOP = FALSE;
        }

    } else {
        printf("No socket connection on %s\nExcecute as root\n", ecat_ifname);
        return FALSE;
    }

    return inOP;
}

void Max_Time_Save(double now_time) {

    if (max_time < now_time) {
        max_time = now_time;
    }
}

static int RS3_write8(uint16 slave, uint16 index, uint8 subindex, uint8 value) {
    int wkc;
    wkc += ec_SDOwrite(slave, index, subindex, FALSE, sizeof (value), &value, EC_TIMEOUTRXM);
    return wkc;
}

static int RS3_write16(uint16 slave, uint16 index, uint8 subindex, uint16 value) {
    int wkc;
    wkc += ec_SDOwrite(slave, index, subindex, FALSE, sizeof (value), &value, EC_TIMEOUTRXM);
    return wkc;
}

static int RS3_write32(uint16 slave, uint16 index, uint8 subindex, uint32 value) {
    int wkc;
    wkc += ec_SDOwrite(slave, index, subindex, FALSE, sizeof (value), &value, EC_TIMEOUTRXM);
    return wkc;
}

double Count2Deg(int Gear_Ratio, INT32 Count) {
    double th = (double) Count * 360 / (2048 * Gear_Ratio);
    return th; // 
}

double Count2DegDot(int Gear_Ratio, INT32 CountPerSec) {
    double th_dot = (double) CountPerSec * 360 / (2048 * Gear_Ratio);
    return th_dot; // [deg/s]
}

double Count2Rad(int Gear_Ratio, INT32 Count) {
    double th = (double) Count * 2 * PI / (2048 * Gear_Ratio);
    return th; // [rad]
}

double Count2RadDot(int Gear_Ratio, INT32 CountPerSec) {
    double th_dot = (double) CountPerSec * 2 * PI / (2048 * Gear_Ratio);
    return th_dot; // [rad]
}

double Count2Rad_ABS(int _Resolution, INT32 Count) {
    double th = (double) Count * 2 * PI / (_Resolution);
    return th;
}

INT32 Count_tf(int _Ratio, INT32 _Count_in) {
    INT32 _Count_out = (INT32) _Count_in / (_Ratio);
    return _Count_out;
}

INT16 Tor2Cur(double OutputTorque, double _Kt, double _ratedCur) {
    INT16 inputCurrent = OutputTorque/_Kt/_ratedCur * 1000;
    
    return inputCurrent;
}
